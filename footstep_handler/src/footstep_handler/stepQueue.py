import copy

class StepQueue(object):
    
    _steps = None
    _nextStep = 0
    
    def __init__(self):
        self._steps = []
        self._nextStep = 0
        return
    
    def clear(self):
         self._steps = []
         self._nextStep = 0
         return
     
    def addStep(self, step):
        new_step = copy.deepcopy(step)
        self._steps.append(new_step)
        return
    
    def addSteps(self, steps):
        #self._steps = copy.deepcopy(steps)
        self._steps.extend(copy.deepcopy(steps) )
        self._nextStep = 0
        return
    
    def isEmpty(self):
        return len(self._steps) == 0
    
    def isFInished(self):
        if self.isEmpty() :
            return True
        if len(self._steps) <= self._nextStep :
            return True
        return False
        
    def size(self):
        return len(self._steps)
    
    def nextStep(self):
        if self.isFInished():
            return None
        
        nextStep = self._steps[self._nextStep]
        self._nextStep = self._nextStep +1
        return nextStep
