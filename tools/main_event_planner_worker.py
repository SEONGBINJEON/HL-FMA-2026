"""One bounded compute job; never owns a device or sends motion commands."""
import queue
import threading
import time


class PlanWorker:
    def __init__(self):self.result=queue.Queue(maxsize=1);self.thread=None;self.closed=False
    def submit(self,kind,key,fn,*args):
        if self.closed or self.thread is not None:return False
        def work():
            start=time.perf_counter()
            try:out={'kind':kind,'key':key,'value':fn(*args),'error':None}
            except Exception as exc:out={'kind':kind,'key':key,'value':None,'error':type(exc).__name__+': '+str(exc)}
            out['compute_s']=time.perf_counter()-start
            if not self.closed:
                try:self.result.put_nowait(out)
                except queue.Full:pass
        self.thread=threading.Thread(target=work,name='main-event-planner',daemon=True);self.thread.start();return True
    def poll(self):
        try:result=self.result.get_nowait()
        except queue.Empty:return None
        self.thread=None;return result
    def close(self):self.closed=True
