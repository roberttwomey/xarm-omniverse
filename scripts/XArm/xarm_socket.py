import ast
import socket
import threading
import carb

class XArmSocket():
    def __init__(self) -> None:
        # sending position data to arm  
        self.txsocket = None
        self.txconn = None
        
        # tracking info
        self.rxsocket = None
        self.rxconn = None

        self.cam_to_nose = None
        self.face_direction = None
        self.dx = None
        self.dy = None
        self.z = None
        self.rx = None
        self.ry = None
        self.rz = None
        self.thistype = "relax"


        # threads
        self.txsocket_thread = None
        self.rxsocket_thread = None

    def start_txsocket(self):
        self.txsocket_thread = threading.Thread(target=self.setup_txsocket)
        self.txsocket_thread.start()

    def start_rxsocket(self):
        self.rxsocket_thread = threading.Thread(target=self.setup_rxsocket)
        self.rxsocket_thread.start()

    def setup_txsocket(self):
        if self.txsocket is None:
            print("tx: setup socket")
            self.txsocket = socket.socket()
            # allow socket to reuse address
            self.txsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            txport = 12345
            self.txsocket.bind(('', txport))
        
        print("tx: listening for connections...")
        # https://docs.python.org/3/library/socket.html#socket.socket.listen
        self.txsocket.listen(5) # number of unaccepted connections allow (backlog)
        
        while True:
            print("tx: waiting for connection...", end="")
            self.txconn, self.txaddr = self.txsocket.accept()
            print("tx: accepted connection from:",str(self.txaddr[0]), ":", str(self.txaddr[1]))

    def setup_rxsocket(self):
        if self.rxsocket is None:
            self.rxsocket = socket.socket()
            # allow socket to reuse address
            self.rxsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            rxport = 12346
            self.rxsocket.bind(('', rxport))
        
        # https://docs.python.org/3/library/socket.html#socket.socket.listen
        self.rxsocket.listen(5) # number of unaccepted connections allow (backlog)
        
        while True:
            print("rx: waiting for connection...", end="")
            self.rxconn, self.rxaddr = self.rxsocket.accept()
            print("rx: accepted connection from:",str(self.rxaddr[0]), ":", str(self.rxaddr[1]))

            while True:
                data = self.rxconn.recv(1024)

                if data == b'': 
                    print("rx: socket closed.")
                    break

                # if data:
                #     message = data.decode()
                #     # carb.log_error("received:" + str(type(message)) + message)
                #     cam_to_nose = [0, 0, 0]
                #     face_direction = [0, 0, 0]
                #     try:
                #         cam_to_nose[0], cam_to_nose[1], cam_to_nose[2], face_direction[0], face_direction[1], face_direction[2], dx, dy = ast.literal_eval(message)
                #     except ValueError:
                #         self.cam_to_nose = None
                #         self.face_direction = None
                #         self.dx = None
                #         self.dy = None
                #     else: 
                #     # print("received:", x, y, z, dx, dy, dist)
                #         weight = 0.1
                #         self.cam_to_nose = cam_to_nose
                #         self.face_direction = face_direction
                #         self.dx = weight*dx
                #         self.dy = weight*dy
                # else:
                #     self.cam_to_nose = None
                #     self.face_direction = None
                #     self.dx = None
                #     self.dy = None

                if data:
                    # RGB Camera
                    message = data.decode()
                    # print("received:", type(message), message)
                    try:
                        thistype, thispayload = ast.literal_eval(message)
                        self.thistype = thistype
                    except:
                        print("rx: problem with data", message)
                        continue
                    
                    if thistype not in ["face", "relax", "rand", "pos"]:
                        print("rx: unknown command", thistype)
                        continue

                    if thistype == "face":
                        y, p, r, dx, dy, dz = thispayload
                        # print("received:", x, y, z, dx, dy)
                        weight = 0.3  # 0.05
                        self.dx = weight*dx
                        self.dy = weight*dy
                        self.dz = weight*dz

                        self.rx = y
                        self.ry = p
                        self.rz = r
                        self.face_direction = [y, p, r]
                    else:
                        self.face_direction = None
                    
                    if thistype == "pos":
                        x, y, z = thispayload
                        self.dx = x
                        self.dy = y
                        self.dz = z

    def shut_down_socket(self):
        if self.txconn:
            try:
                # self._conn.send("Done".encode())
                self.txsocket.shutdown(socket.SHUT_RDWR)
                self.txsocket.close()
                self.txsocket = None
            except socket.error as e:
                pass
        if self.rxconn:
            try:
                # self._conn.send("Done".encode())
                self.rxsocket.shutdown(socket.SHUT_RDWR)
                self.rxsocket.close()
                self.rxsocket = None
            except socket.error as e:
                pass