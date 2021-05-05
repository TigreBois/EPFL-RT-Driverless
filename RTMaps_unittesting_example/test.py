import pytest
import unittest
import mock
import pickle
import sys
import fakeBaseComponent
import fakeRTMaps
import fakeTypes
import fakeCore
import fakeReading_policy
sys.modules["rtmaps"] = fakeRTMaps
sys.modules["rtmaps.types"] = fakeTypes
sys.modules["rtmaps.core"] = fakeCore
sys.modules["rtmaps.reading_policy"] = fakeReading_policy
sys.modules["rtmaps.base_component.BaseComponent"] = fakeBaseComponent

#Import the module to be tested here
import source as s

#Usage: if you want to test automatically the module with input/output pickle files, launch this script using unittest decorator: 'python -m unittest'
#To test it manually, feeding input via the console, launch it like a normal python script: 'python test.py'
#Note that the manual mode's input are strings directly taken from the console; you probably want to process it before feeding it to the module,
#you can do that in the "feedManualInput" function, in the "MockInput" class. 
#To stop the program while testing manually, enter "stop", or cancel the script as usual(with ctrl+c for instance)

inputFile = "input.pickle"
outputFile = "output.pickle"
manual_mode = False
#inputLIst is the list of input that's going to be fed to the module; outputLIst is the list of output used to check the module's behaviour.
#If you need to do special handling of the pickle file, do it here.
try:
    with open(inputFile, 'rb') as handle:
        inputList = pickle.load(handle)
    with open(outputFile, 'rb') as handle:
        outputList = pickle.load(handle)
#If you're testing in manual mode, you don't need input/output pickle files; but otherwise it will throw an exception later
except FileNotFoundError:
    inputList = None
    outputList = None






class MockInput():
    def __init__(self):
        if not manual_mode:
            self.ioelt = inputList.pop(0)
        else:
            self.ioelt = self.feedManualInput()
    def feedManualInput(self):
        y = input("Enter next input to the module:")
        if y == "stop":
            sys.exit("Program stopped by stop input")
        return y
    


class MockOutput():        
    def write(out):
        if not manual_mode:
            tc = unittest.TestCase()
            tc.assertEqual(out,outputList.pop(0))
        else:
            print("Result in fake output: " + str(out))
        return




class Test(unittest.TestCase):

    

    @classmethod
    def setUpClass(self):
        return
    
    
    def test_Module(self):
        bc = fakeBaseComponent.BaseComponent()
        rtmapspy = s.rtmaps_python()
        rtmapspy.Birth()
        self.module_loop(rtmapspy)
        
    @pytest.mark.timeout(6)    
    def module_loop(self,module):
        with mock.patch.dict("source.rtmaps_python.outputs", {"out" : MockOutput}):
            if not manual_mode:
                if inputList == None or outputList == None:
                    raise FileNotFoundError("Input or Output pickle file not found !")
                print("Input list: " + str(inputList))
                print("Output list: " + str(outputList))
                while inputList:
                    mi = MockInput()
                    with mock.patch.dict("source.rtmaps_python.inputs", {"in": mi}):
                        module.Core()
            else:
                while True:
                    mi = MockInput()
                    with mock.patch.dict("source.rtmaps_python.inputs", {"in": mi}):
                        module.Core()
        
    

    

if __name__ == '__main__':
    manual_mode = True
    unittest.main()

        