# ntrip_client.py
import requests
from requests.auth import HTTPBasicAuth

class NTRIPClient:
    def __init__(self, host, port, mountpoint, user, password):
        self.url = f"http://{host}:{port}/{mountpoint}"
        self.auth = HTTPBasicAuth(user, password)
        self.session = requests.Session()
        self.headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP EmlidClient/1.0',
            'Connection': 'close',
            'Accept': '*/*'
        }

    def get_corrections(self):
        """Stream RTCM correction data from NTRIP caster."""
        try:
            response = self.session.get(
                self.url,
                auth=self.auth,
                stream=True,
                headers=self.headers,
                timeout=10
            )
            response.raise_for_status()

            print("✅ Connected to NTRIP caster")
            for chunk in response.iter_content(chunk_size=1024):
                if chunk:
                    yield chunk  # RTCM data bytes

        except Exception as e:
            print(f"❌ NTRIP error: {e}")
