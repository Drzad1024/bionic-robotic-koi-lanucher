import sys
from PyQt6.QtWidgets import QApplication, QMainWindow
from Lanucher_UI import *
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)


class MyWindows(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(MyWindows,self).__init__()
        self.setupUi(self)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindows()
    window.show()
    sys.exit(app.exec())

