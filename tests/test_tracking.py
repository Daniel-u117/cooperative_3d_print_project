import contextlib
import io
from pathlib import Path
import queue
import tempfile
import unittest
from unittest.mock import Mock, patch

from util.job_tracking import instrument_job, START_VARIABLE, DONE_VARIABLE
from util.tracking import track


class ClockStop:
    def __init__(self):
        self.now = 0.0
        self.stopped = False

    def is_set(self):
        return self.stopped

    def set(self):
        self.stopped = True

    def wait(self, seconds):
        self.now += seconds
        if self.now > 2:
            self.stopped = True


def sample(status="processing", e=1, x=100, started=True, done=False):
    return dict(status=status, e=e, x=x, started=started, done=done)


class TrackingTests(unittest.TestCase):
    def run_tracker(self, samples, *, expected_error=None, queue_full_once=False):
        stop = ClockStop()
        errors, controls, acknowledgements, events = (queue.Queue() for _ in range(4))
        coords, footprints, samples_read, calls = {}, {}, {1: 0, 2: 0}, []
        tokens = {1: 'job-one', 2: 'job-two'}
        for p in samples:
            controls.put(('hold', p))
            controls.put(('print', p, {'file': f'{p}_m.gcode', 'token': tokens[p]}))
        publications = []
        original_put = events.put_nowait
        failed = False

        def event(item):
            nonlocal failed
            p, _ = item
            if queue_full_once and p in samples and samples_read[p] > 0 and not failed:
                failed = True
                raise queue.Full
            publications.append((item, coords[p], footprints[p]))
            original_put(item)

        events.put_nowait = event

        def get(url, **kwargs):
            calls.append((url, kwargs))
            p = 1 if 'robot-a' in url else 2
            sequence = samples[p]
            frame = sequence[min(samples_read[p], len(sequence) - 1)]
            if kwargs['params'].get('key') == 'global':
                result = {
                    START_VARIABLE: tokens[p] if frame['started'] is True else frame['started'],
                    DONE_VARIABLE: tokens[p] if frame['done'] is True else frame['done'],
                }
            else:
                samples_read[p] += 1
                result = {'state': {'status': frame['status']},
                          'move': {'axes': [{'machinePosition': frame['x']}, {'machinePosition': 100}],
                                   'extruders': [{'position': frame['e']}]}}
            return Mock(json=lambda: {'result': result})

        output = io.StringIO()
        with patch('util.tracking.read_status', side_effect=lambda *a, **kw: (['I', 'I'], [(300, 0), (300, 0)])), \
             patch('util.tracking.requests.get', side_effect=get), \
             patch('util.tracking.time.monotonic', side_effect=lambda: stop.now), \
             patch('util.tracking.WATCH_TIMEOUT', 0.8), \
             patch('util.tracking.COMPLETION_TIMEOUT', 0.4), contextlib.redirect_stdout(output):
            args = ({1: 'robot-a', 2: 'robot-b'}, coords, events, controls, acknowledgements,
                    stop, errors, queue.Queue(maxsize=1))
            kwargs = dict(printer={1: None, 2: None}, store_polygons=footprints,
                          polygon_at_position=lambda p, printers, pos: pos)
            if expected_error:
                with self.assertRaisesRegex((TimeoutError, RuntimeError), expected_error):
                    track(*args, **kwargs)
            else:
                track(*args, **kwargs)
        return list(acknowledgements.queue), publications, errors, output.getvalue(), samples_read, calls

    def test_missed_printing_samples_complete_with_job_token_and_final_pose(self):
        acks, events, errors, output, _, _ = self.run_tracker({
            1: [sample(status='idle', e=8.2, x=119.166, done=True)]})
        self.assertEqual(acks, [('Completed', 1)])
        final = [entry for entry in events if entry[0] == (1, 'free')][1:]
        self.assertEqual(final, [((1, 'free'), (119.166, 100.0), (119.166, 100.0))])
        self.assertTrue(errors.empty())
        self.assertIn('printing_sample_missed=True', output)

    def test_moving_extrusion_acknowledges_then_completes_once(self):
        acks, events, errors, _, _, _ = self.run_tracker({1: [
            sample(e=1, x=101), sample(e=2, x=102), sample(e=3, x=103),
            sample(status='idle', e=3, x=104, done=True)]})
        self.assertEqual(acks, [('Is printing', 1)])
        self.assertEqual([event[1] for event in events if event[0] == (1, 'free')],
                         [(300.0, 0.0), (104.0, 100.0)])
        self.assertTrue(errors.empty())

    def test_watching_one_robot_does_not_starve_other_completion(self):
        acks, events, _, _, reads, calls = self.run_tracker({
            1: [sample(e=1, started=False)],
            2: [sample(status='idle', x=170, done=True)]}, expected_error='no moving extrusion')
        self.assertEqual(acks, [('Completed', 2)])
        self.assertTrue(any(event[0] == (2, 'free') and event[1] == (170.0, 100.0) for event in events))
        self.assertGreater(reads[1], 1)
        self.assertGreater(reads[2], 0)
        self.assertTrue(all(url.endswith('/rr_model') for url, _ in calls))

    def test_idle_and_stale_token_never_complete(self):
        for started in (False, 'previous-job'):
            with self.subTest(started=started):
                acks, events, _, _, _, _ = self.run_tracker({
                    1: [sample(status='idle', started=started, done='previous-job')]},
                    expected_error='no moving extrusion')
                self.assertEqual(acks, [])
                self.assertEqual(len([e for e in events if e[0] == (1, 'free')]), 1)

    def test_started_job_idle_without_completion_is_not_assumed_successful(self):
        acks, events, _, _, _, _ = self.run_tracker({1: [sample(status='idle')]},
                                                   expected_error='idle without its completion token')
        self.assertEqual(acks, [])
        self.assertEqual(len([e for e in events if e[0] == (1, 'free')]), 1)

    def test_completion_token_alone_does_not_release_busy_robot(self):
        acks, events, _, _, _, _ = self.run_tracker({1: [sample(status='busy', done=True)]},
                                                   expected_error='no moving extrusion')
        self.assertEqual(acks, [])
        self.assertEqual(len([e for e in events if e[0] == (1, 'free')]), 1)

    def test_full_event_queue_retries_without_duplicate_acknowledgement(self):
        acks, events, errors, _, _, _ = self.run_tracker({1: [sample(status='idle', done=True)]},
                                                        queue_full_once=True)
        self.assertEqual(acks, [('Completed', 1)])
        self.assertEqual(len([e for e in events if e[0] == (1, 'free')]), 2)
        self.assertTrue(errors.empty())

    def test_stationary_priming_does_not_confirm_printing(self):
        acks, _, _, _, _, _ = self.run_tracker({1: [sample(e=e) for e in range(20)]},
                                              expected_error='no moving extrusion')
        self.assertEqual(acks, [])

    def test_paused_robot_is_not_resumed_by_polling(self):
        acks, _, _, _, _, calls = self.run_tracker({1: [sample(status='paused')]},
                                                  expected_error='no moving extrusion')
        self.assertEqual(acks, [])
        self.assertTrue(all(url.endswith('/rr_model') for url, _ in calls))

    def test_job_tokens_are_unique_and_completion_follows_final_motion(self):
        with tempfile.TemporaryDirectory() as temp:
            tokens = []
            for name in ('one.gcode', 'two.gcode'):
                path = Path(temp) / name
                path.write_text('G1 X100 Y100 E1\nG1 Z10\n')
                token = instrument_job(path)
                tokens.append(token)
                text = path.read_text()
                self.assertIn(f'set global.{START_VARIABLE} = "{token}"', text)
                self.assertTrue(text.endswith(f'M400\nset global.{DONE_VARIABLE} = "{token}"\n'))
                self.assertLess(text.index('G1 Z10'), text.rindex('M400'))
            self.assertNotEqual(tokens[0], tokens[1])


if __name__ == '__main__':
    unittest.main()
