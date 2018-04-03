
from PyQt5 import QtCore, QtGui, QtWidgets
from configurator import Ui_configurator
import generator as gen


class ShipHolderApplication(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(ShipHolderApplication, self).__init__(parent)
        self.createWidgets()
        self.connectActions()

    def createWidgets(self):
        self.ui = Ui_configurator()
        self.ui.setupUi(self)
		
    def connectActions(self):
        self.ui.generate_buttom.clicked.connect(self.generate_click)
        self.ui.actionAbout.triggered.connect(self.about_click)
        self.ui.actionSave.triggered.connect(self.save_click)
        self.ui.actionLoad.triggered.connect(self.load_click)
        self.ui.path_button.clicked.connect(self.path_click)


    def generate_click(self):
        mount_type = self.ui.mount_combo.currentText()
        board_type = self.ui.board_combo.currentText()
        max_rate = self.ui.maxrate_spin.value()
        pec = self.ui.pec_spin.value()
        auto_sid = self.ui.comboBox_11.currentText()

        worm1 = self.ui.axis1_worm_spindouble.value()
        gear1 = self.ui.axis1_gear_spindouble.value()
        stepper1 = self.ui.axis1_motor_combo.currentText()
        micro1 = self.ui.axis1_micro_combo.currentText()
        slew1 = self.ui.axis1_mod_combo.currentText()
        driver1 = self.ui.axis1_driver_combo.currentText()
        
        worm2 = self.ui.axis2_worm_spindouble.value()
        gear2 = self.ui.axis2_gear_spindouble.value()
        stepper2 = self.ui.axis2_motor_combo.currentText()
        micro2 = self.ui.axis2_micro_combo.currentText()
        slew2 = self.ui.axis2_mod_combo.currentText()
        driver2 = self.ui.axis2_driver_combo.currentText()

        gen.test_config(mount_type, board_type, max_rate, pec, auto_sid,\
            worm1, gear1, stepper1, micro1, slew1, driver1, worm2, gear2,\
            stepper2, micro2, slew2, driver2)

    def path_click(self):
        self.PathName = QtWidgets.QFileDialog.getSaveFileName( self, 
			"Define Save File for OnStep Conf", QtCore.QDir.homePath(), 
			"text File (*.txt)")
        self.ui.path_line.setText(self.PathName[0])
        print(self.PathName[0])

    def about_click(self):
        QtWidgets.QMessageBox.about(self,'About',"Onstep configurator GUI\n about sebastien Durand")

    def save_click(self):
        self.SaveName = QtWidgets.QFileDialog.getSaveFileName( self, 
			"Definir nom de sauvegarde", QtCore.QDir.homePath(), 
			"Fichiers texte (*.txt)")
        print(self.SaveName[0]) 

    def load_click(self):
        self.LoadName = QtWidgets.QFileDialog.getOpenFileName( self, 
			"Ouvrir un fichier monture", QtCore.QDir.homePath())
        print(self.LoadName[0])

    def main(self):
        self.show()

if __name__ == "__main__":
	import sys
	app = QtWidgets.QApplication(sys.argv)
	#conf = QtWidgets.QMainWindow()
	myapp = ShipHolderApplication()
	myapp.main()
	#QTimer.singleShot(1,myapp.createWidgets())
	sys.exit(app.exec_())

