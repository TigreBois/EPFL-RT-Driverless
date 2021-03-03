import unittest
from src.main import Example
class ExampleTest(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        global ex 
        ex = Example()

    #should pass
    def testMain_right(self):
        self.assertEqual(ex.main(),"Hello World!")
    
    def test_add(self):
        self.assertEqual(ex.add(1,2),3)

    

if __name__ == '__main__':
    unittest.main()