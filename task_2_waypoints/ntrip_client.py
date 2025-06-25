import requests
from requests.auth import HTTPBasicAuth
import time
import logging

class NTRIPClient:
    def __init__(self, host, port, mountpoint, user, password):
        if not all([host, port, mountpoint]):
            raise ValueError("Host, port and mountpoint are required")
        self.url = f"http://{host}:{port}/{mountpoint}"
        self.auth = HTTPBasicAuth(user, password)
        self.session = requests.Session()
        self.connected = False
        self.last_data_time = time.time()

        self.headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP EmlidClient/1.0',
            'Connection': 'keep-alive',  # Changed from 'close'
            'Accept': '*/*'
        }

    def get_corrections(self):
        retry_count = 0
        max_retries = 3
        while retry_count < max_retries:
            try:
                response = self.session.get(
                    self.url,
                    auth=self.auth,
                    stream=True,
                    headers=self.headers,
                    timeout=10
                )
                response.raise_for_status()
                self.connected = True
                print("✅ Connected to NTRIP caster")
                
                for chunk in response.iter_content(chunk_size=1024):
                    if chunk:
                        yield chunk
                        
            except requests.exceptions.RequestException as e:
                retry_count += 1
                print(f"❌ NTRIP error (attempt {retry_count}/{max_retries}): {e}")
                if retry_count < max_retries:
                    time.sleep(5)  # Wait before retry
                continue
            except Exception as e:
                print(f"❌ Unexpected error: {e}")
                break
    def connect(self, max_retries=3, retry_delay=2):
        """Connect to NTRIP server with retries"""
        retry_count = 0
        while retry_count < max_retries:
            try:
                response = self.session.get(
                    self.url,
                    auth=self.auth,  # Use stored auth
                    headers=self.headers,  # Use stored headers
                    stream=True,
                    timeout=10
                )
                if response.status_code == 200:
                    self.connected = True
                    logging.info("✅ NTRIP connection successful")
                    return True
                
                logging.warning(f"Connection attempt {retry_count + 1} failed: {response.status_code}")
                retry_count += 1
                time.sleep(retry_delay)
            except Exception as e:
                logging.error(f"NTRIP connection error: {e}")
                retry_count += 1
                time.sleep(retry_delay)
        
        self.connected = False
        logging.error("Failed to connect after all retries")
        return False

    def cleanup(self):
        """Clean up resources safely"""
        try:
            if self.session:
                self.session.close()
            self.connected = False
            logging.info("NTRIP client cleaned up")
        except Exception as e:
            logging.error(f"Error during NTRIP cleanup: {e}")    



    def check_connection(self):
        """Check if connection is alive"""
        try:
            if not self.session:
                return False
            response = self.session.get(self.url, timeout=5, stream=True)
            return response.status_code == 200
        except:
            return False
        