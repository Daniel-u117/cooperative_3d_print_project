import subprocess
import requests
import re
import socket
import time


# run_netsh("disconnect", f'interface=Wi-Fi') --> this will disrupt the connection on purpose




def get_status(retries=5):
    url1 = "http://192.168.0.15/rr_status"
    url2 = "http://192.168.0.16/rr_status"
    for attempt in range(retries):
        try:
            r1 = requests.get(url1, params={"type": 0}, timeout=5)
            r1.raise_for_status()
            r2 = requests.get(url2, params={"type": 0}, timeout=5)
            r2.raise_for_status()

            data1 = r1.json()
            data2 = r2.json()

            s1 = data1["status"]
            cm1 = data1["coords"]["machine"]
            s2 = data2["status"]
            cm2 = data2["coords"]["machine"]

            if s1 is None or cm1 is None or s2 is None or cm2 is None:
                continue

 
            return [[s1, s2],[cm1, cm2]]
        except (requests.exceptions.RequestException, ConnectionError) as error:
            print(f'{attempt} attempt to reconnect')
            rec = reconnect_to_ambots()
            
            if rec:
                continue
            else:
                raise

    raise ConnectionError(
        f"Could not obtain valid status from both printers after {retries} attempts"
    )



def run_netsh(*args, timeout=15):
    result = subprocess.run(
        ["netsh", "wlan", *args],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="ignore",
        timeout=timeout
    )
    return result


def current_wifi():
    result = run_netsh("show", "interfaces")
    text = result.stdout

    state_match = re.search(r"^\s*State\s*:\s*(.+)$", text, re.MULTILINE)
    ssid_match = re.search(r"^\s*SSID\s*:\s*(.+)$", text, re.MULTILINE)

    state = state_match.group(1).strip() if state_match else None
    ssid = ssid_match.group(1).strip() if ssid_match else None

    return state, ssid, text





def tcp_reachable(ip, port, timeout=2):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False



def reconnect_to_ambots(max_wait=45):
    WIFI_SSID = "AMBOTS_5G"
    WIFI_PROFILE = "AMBOTS_5G"      # usually same as SSID
    WIFI_INTERFACE = "Wi-Fi"        # check with: netsh wlan show interfaces

    PRINTER_IP = "192.168.0.15"
    PRINTER_PORT = 80
    state, ssid, _ = current_wifi()

    if state and state.lower() == "connected" and ssid == WIFI_SSID:
        if tcp_reachable(PRINTER_IP, PRINTER_PORT):
            con1 = requests.get('http://192.168.0.15/rr_connect?password=reprap', timeout = 0.5)

            con2 = requests.get('http://192.168.0.16/rr_connect?password=reprap', timeout = 0.5)
            return True

    print(f"Current Wi-Fi: state={state}, ssid={ssid}")
    print(f"Reconnecting to {WIFI_SSID}...")

    # Optional: force disconnect first
    #run_netsh("disconnect", f'interface={WIFI_INTERFACE}')

    

    result = run_netsh(
        "connect",
        f"name={WIFI_PROFILE}",
        f"ssid={WIFI_SSID}",
        f'interface={WIFI_INTERFACE}'
    )
   
    print("netsh stdout:", result.stdout.strip())
    print("netsh stderr:", result.stderr.strip())
    time.sleep(3)

    start = time.perf_counter()
    dur = 60

    while (state.lower() != "connected" or ssid != WIFI_SSID or not tcp_reachable(PRINTER_IP, PRINTER_PORT)) and (time.perf_counter() - start < dur):
        print(time.perf_counter() - start<dur)
        print(state.lower() != 'connected')
        print(ssid != WIFI_SSID)
        print(not tcp_reachable(PRINTER_IP, PRINTER_PORT))

        
        print('another attempt')

        result = run_netsh(
        "connect",
        f"name={WIFI_PROFILE}",
        f"ssid={WIFI_SSID}",
        f'interface={WIFI_INTERFACE}'
        )
        print(f"Waiting... state={state}, ssid={ssid}")
        time.sleep(3)
        state, ssid, _ = current_wifi()
        if state and state.lower() == "connected" and ssid == WIFI_SSID:
            if tcp_reachable(PRINTER_IP, PRINTER_PORT):
                print(f"Reconnected to {WIFI_SSID}; printer is reachable.")
                con1 = requests.get('http://192.168.0.15/rr_connect?password=reprap', timeout = 0.5)
                con2 = requests.get('http://192.168.0.16/rr_connect?password=reprap', timeout = 0.5)
                return True
   
        print("netsh stdout:", result.stdout.strip())
        print("netsh stderr:", result.stderr.strip())
       
    
    raise ConnectionError(f"Could not reconnect to {WIFI_SSID} and reach {PRINTER_IP}.")


# run_netsh("disconnect", f'interface=Wi-Fi') --> this will disrupt the connection on purpose

