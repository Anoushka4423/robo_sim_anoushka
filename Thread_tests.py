import threading
import logging
    

def thread_target():
    logging.info("Inside the thread")
    

def main():
    x = threading.Thread(target=thread_target, daemon=True)
    x.start()
    
    while(True):
        logging.info("In Main")
    