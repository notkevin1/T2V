import bluetooth

class Car:
    def __init__(self, btAddress, RFIDid):
        self.btAddress = btAddress
        self.RFIDid = RFIDid
        self.btSocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    def connect(self):
        try:
            self.btSocket.connect((self.btAddress, 1))
        except Exception as e:
            print(e)
    
    def send(self, command):
        try:
            self.btSocket.send('$' + command + '#')
        except Exception as e:
            print(e)

    def getBTAddress(self):
        return self.btAddress

    def getRFIDid(self):
        return self.RFIDid