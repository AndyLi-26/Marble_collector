import multiprocessing
import time
class A:
    def __init__(self):
        self.time=multiprocessing.Array('f', [0,0])
        
    def pTime(self,temp):
        while 1:
            print("ppp:",temp[:])
        
    def genTime(self,temp):
        while 1:
            temp[0] = time.monotonic()
            temp[1] = time.monotonic()+1
            #self.time[0] = time.strftime("%H:%M:%S", t)
            #print("gen:")
            print("gen:",temp[:])
        
    def run(self):
        pget=multiprocessing.Process(target=self.genTime,args=(self.time,))
        pprt=multiprocessing.Process(target=self.pTime,args=(self.time,))
        pprt.start()
        pget.start()
        time.sleep(5)
        pprt.terminate()
        pget.terminate()
if __name__=="__main__":
    a=A()
    a.run()