from ws4py.client.threadedclient import WebSocketClient
from time import sleep


class TestClient(WebSocketClient):
    def opened(self):
        print('opened')
        for i in range(0, 100):
            self.send(str(i))
        sleep(10)
        self.close()

    def closed(self, code, reason):
        print(("Closed down", code, reason))

    def received_message(self, m):
        print("=> %d %s" % (len(m), str(m)))
        if len(m) == 175:
            self.close(reason='Bye bye')

if __name__ == '__main__':
    try:
        # url = 'wss://echo.websocket.org'
        # url = 'ws://10.3.6.140:8085/websocket/data/145/9f762eea-4f12-11e6-91d8-4c34889fd91d/websocket'
        url = 'wss://10.3.6.140:8084/websocket/data/145/9f762eea-4f12-11e6-91d8-4c34889fd91d/websocket'
        # url = 'ws://demo.virtualfortknox.de/msb2/websocket/data/145/9f762eea-4f12-11e6-91d8-4c34889fd91d/websocket'
        ws = TestClient(url, protocols=['http-only', 'chat'])
        ws.daemon = False
        ws.connect()
    except KeyboardInterrupt:
        ws.close()
