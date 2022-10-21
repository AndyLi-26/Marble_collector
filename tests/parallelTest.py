import multiprocessing
import time
class A:
    def __init__(self):
        self.time=multiprocessing.Array('f', [0,0])
        
    def pTime(self,temp):
        while 1:
            print("ppp:",temp[:])
        
    def genTime(self,temp):
        for i in range(100):
            temp[0] = time.monotonic()
            temp[1] = time.monotonic()+1
            #self.time[0] = time.strftime("%H:%M:%S", t)
            #print("gen:")
            time.sleep(1)
            print("gen:",temp[:])
            print("currenti:",i)
        print("dead"*50)
        
    def run(self):
        pget=multiprocessing.Process(target=self.genTime,args=(self.time,))
        pprt=multiprocessing.Process(target=self.pTime,args=(self.time,))
        pprt.start()
        pget.start()

        pget.join(5)
        pprt.terminate()
        
if __name__=="__main__":
    a=A()
    a.run()