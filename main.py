
from PyQt5 import QtCore, QtWidgets
from configurator import Ui_configurator
import generator as gen


class ShipHolderApplication(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(ShipHolderApplication, self).__init__(parent)
        self.createWidgets()
        self.connectActions()
        self.PathName=''

    def createWidgets(self):
        self.ui = Ui_configurator()
        self.ui.setupUi(self)
		
    def connectActions(self):
        self.ui.generate_buttom.clicked.connect(self.generate_click)
        self.ui.actionAbout.triggered.connect(self.about_click)
        self.ui.actionSave.triggered.connect(self.save_click)
        self.ui.actionLoad.triggered.connect(self.load_click)
        self.ui.path_button.clicked.connect(self.path_click)
        self.ui.buzzer_type_comboBox.currentIndexChanged.connect(self.buz_type_change)
        self.ui.buzzer_checkBox.stateChanged.connect(self.buz_change)
    def buz_change(self):
        if self.ui.buzzer_checkBox.isChecked():
            if self.ui.buzzer_type_comboBox.currentText() == "Speaker":
                self.ui.freq_spinBox.setEnabled(True)
                self.ui.default_sound_checkBox.setEnabled(True)
                self.ui.label_19.setEnabled(True)
        else:
            self.ui.freq_spinBox.setEnabled(False)
            self.ui.default_sound_checkBox.setEnabled(False)
            self.ui.label_19.setEnabled(False)
        
    def buz_type_change(self):

        if self.ui.buzzer_type_comboBox.currentText() == "Buzzer":
            self.ui.freq_spinBox.setEnabled(False)
            self.ui.default_sound_checkBox.setEnabled(False)
            self.ui.label_19.setEnabled(False)
            
        elif self.ui.buzzer_type_comboBox.currentText() == "Speaker":
            self.ui.freq_spinBox.setEnabled(True)
            self.ui.default_sound_checkBox.setEnabled(True)
            self.ui.label_19.setEnabled(True)
            
    def generate_click(self):
        var = self.call_var()
        temp_path = "config_file_temp.conf"
        self.create_conf_file(temp_path,var)
        self.ui.prompt_textbrowser.append("Generation Start")
        if self.PathName == '':
            gen.onstep_config(temp_path)
        else:
            gen.onstep_config(temp_path, path=self.PathName)
        self.ui.prompt_textbrowser.append("Generation End succeful")

    def path_click(self):
        self.PathName = QtWidgets.QFileDialog.getSaveFileName( self, 
			"Define Save File for OnStep Conf", QtCore.QDir.homePath(), 
			"text File (*.txt)")
        self.ui.path_line.setText(self.PathName[0])
        self.PathName = self.PathName[0]
        self.ui.prompt_textbrowser.append("Config File path : "+self.PathName)

    def about_click(self):
        QtWidgets.QMessageBox.about(self,'About',"Onstep configurator GUI v.BETA_0.6\n About Sébastien Durand\n For help contact me at : dragonlost1@hotmail.fr")
        
    def call_var(self):
        # General ____________________________________________
        board_type = self.ui.board_combo.currentText()
        mount_type = self.ui.mount_combo.currentText()
        max_rate = self.ui.maxrate_spin.value()
        auto_sid = self.ui.auto_sid_tracking_checkBox.isChecked()

        worm1 = self.ui.axis1_worm_spindouble.value()
        gear1 = self.ui.axis1_gear_spindouble.value()
        stepper1 = self.ui.axis1_motor_spin.value()
        micro1 = self.ui.axis1_micro_combo.currentText()
        slew1 = self.ui.axis1_mod_combo.currentText()
        driver1 = self.ui.axis1_driver_combo.currentText()
        reverse1 = self.ui.axis1_reverse_checkBox.isChecked()
        ena1 = self.ui.axis1_enable_comboBox.currentText()
        fault1 = self.ui.axis1_fault_comboBox.currentText()
        
        worm2 = self.ui.axis2_worm_spindouble.value()
        gear2 = self.ui.axis2_gear_spindouble.value()
        stepper2 = self.ui.axis2_motor_spin.value()
        micro2 = self.ui.axis2_micro_combo.currentText()
        slew2 = self.ui.axis2_mod_combo.currentText()
        driver2 = self.ui.axis2_driver_combo.currentText()
        reverse2 = self.ui.axis2_reverse_checkBox.isChecked()
        ena2 = self.ui.axis2_enable_comboBox.currentText()
        fault2 = self.ui.axis2_fault_comboBox.currentText()
        
        # Axis3/2 ____________________________________________
        rot3 = self.ui.rot_checkBox.isChecked()
        foc1 = self.ui.focus1_checkBox.isChecked()
        foc2 = self.ui.focus2_checkBox.isChecked()
        
        rot_rate = self.ui.rot_mrate_spinBox.value()
        rot_step = self.ui.rot_step_spinBox.value()
        rot_micro = self.ui.rot_mode_comboBox.currentText()
        rot_gear = self.ui.rot_gear1_doubleSpinBox.value()
        rot_gear_2 = self.ui.rot_gear2_doubleSpinBox.value()
        rot_reverse = self.ui.rot_reverse_checkBox.isChecked()
        rot_min_degr = self.ui.rot_min_angle_doubleSpinBox.value()
        rot_max_degr = self.ui.rot_max_angle_doubleSpinBox.value()
        rot_disable = self.ui.rot_di_comboBox.currentText()
        
        foc1_rate = self.ui.focus1_mrate_spinBox.value()
        foc1_ratio = self.ui.focus1_micro_doubleSpinBox.value()
        foc1_reverse = self.ui.focus1_reverse_checkBox.isChecked()
        foc1_min_mm = self.ui.focus1_min_mil_doubleSpinBox.value()
        foc1_max_mm = self.ui.focus1_max_mil_doubleSpinBox.value()
        focus1_disable = self.ui.focus1_di_comboBox.currentText()
        
        foc2_rate = self.ui.focus2_mrate_spinBox.value()
        foc2_ratio = self.ui.focus2_micro_doubleSpinBox.value()
        foc2_reverse = self.ui.focus2_reverse_checkBox.isChecked()
        foc2_min_mm = self.ui.focus2_min_mil_doubleSpinBox.value()
        foc2_max_mm = self.ui.focus2_max_mil_doubleSpinBox.value()
        focus2_disable = self.ui.focus2_di_comboBox.currentText()
        
        # option ____________________________________________
        baud = self.ui.serial_baud_comboBox.currentText()
        baud4 = self.ui.serial4_baud_comboBox.currentText()
        
        esp = self.ui.esp_checkBox.isChecked()
        
        pec = self.ui.pec_checkBox.isChecked()
        pec_pul = self.ui.pec_pul_checkBox.isChecked()
        pec_buffer = self.ui.pec_buffer_spin.value()
        
        goto_assist = self.ui.goto_assist_checkBox.isChecked()
        
        strict_park = self.ui.strict_checkBox.isChecked()
        
        st4 = self.ui.st4_checkBox.isChecked()
        st4_pul = self.ui.st4_pul_checkBox.isChecked()
        alt_st4 = self.ui.alternative_st4_checkBox.isChecked()
        hand = self.ui.hand_checkBox.isChecked()
        
        pulse = self.ui.separate_pulse_checkBox.isChecked()
        
        guide_time = self.ui.guide_time_spinBox.value()
        
        rtc = self.ui.rtc_combo.currentText()
        
        pps = self.ui.pps_checkBox.isChecked()
        pps_pul = self.ui.pps_pul_checkBox.isChecked()
        
        pec_set = self.ui.pec_set_checkBox.isChecked()
        analog_pec = self.ui.analog_pec_spinBox.value()
        pec_logic = self.ui.pec_logic_comboBox.currentText()
        
        limit = self.ui.limit_checkBox.isChecked()
        
        led1 = self.ui.state_led_checkBox.isChecked()
        
        led2 = self.ui.state_led2_checkBox.isChecked()
        led2_intensity = self.ui.intensity_led2_spinBox.value()
        reticule = self.ui.reticule_checkBox.isChecked()
        ret_intensity = self.ui.intensity_ret_spinBox.value()
        
        buzzer = self.ui.buzzer_checkBox.isChecked()
        buzzer_type = self.ui.buzzer_type_comboBox.currentText()
        freq_sound = self.ui.freq_spinBox.value()
        def_sound = self.ui.default_sound_checkBox.isChecked()
        
        atmos = self.ui.atmos_checkBox.isChecked()
        
        mem_flip_mer = self.ui.save_mer_flip_checkBox.isChecked()
        
        home_pause = self.ui.home_pause_checkBox.isChecked()
        
        mem_max_rate = self.ui.max_rate_save_checkBox.isChecked()
        
        accel = self.ui.degre_accel_doubleSpinBox.value()
        
        rapid_stop = self.ui.degre_rapid_stop_doubleSpinBox.value()
        
        backlash = self.ui.backlash_rate_spinBox.value()
        
        off_axis2 = self.ui.off_axis2_checkBox.isChecked()
        
        degre_e = self.ui.degre_e_spinBox.value()
        degre_w = self.ui.degre_w_spinBox.value()
        
        min_dec = self.ui.min_dec_spinBox.value()
        max_dec = self.ui.max_dec_spinBox.value()
        
        pol_limit = self.ui.pol_limit_spinBox.value()
        
        max_az = self.ui.max_az_spinBox.value()
        
        #________________________________________________________________
        
        var = [board_type, mount_type, max_rate, auto_sid, worm1, 
               gear1, stepper1, micro1, slew1, driver1, reverse1, ena1, fault1,
               worm2, gear2, stepper2, micro2, slew2, driver2, reverse2, ena2,
               fault2, rot3, foc1, foc2, rot_rate, rot_step, rot_micro, 
               rot_gear, rot_gear_2, rot_reverse, rot_min_degr, rot_max_degr,
               rot_disable, foc1_rate, foc1_ratio, foc1_reverse, foc1_min_mm,
               foc1_max_mm, focus1_disable, foc2_rate, foc2_ratio, foc2_reverse,
               foc2_min_mm, foc2_max_mm, focus2_disable, baud, baud4, esp, pec,
               pec_pul, pec_buffer, goto_assist, strict_park, st4, st4_pul, 
               alt_st4, hand, pulse, guide_time, rtc, pps, pps_pul, pec_set,
               analog_pec, pec_logic, limit, led1, led2, led2_intensity, 
               reticule, ret_intensity, buzzer, buzzer_type, freq_sound, 
               def_sound, atmos, mem_flip_mer, home_pause, mem_max_rate, accel,
               rapid_stop, backlash, off_axis2, degre_e, degre_w, min_dec, 
               max_dec, pol_limit, max_az]
        
        return var
    
    def save_click(self):
        self.SaveName = QtWidgets.QFileDialog.getSaveFileName( self, 
			"Save Conf File : ", QtCore.QDir.homePath(), 
			"Fichiers texte (*.txt)")
        if self.SaveName[0]=='':
            self.ui.prompt_textbrowser.append("No Save"+self.SaveName[0])
        else:
            self.ui.prompt_textbrowser.append("Save Conf File : "+self.SaveName[0])
            var = self.call_var()
            self.create_conf_file(self.SaveName[0],var)

    def load_click(self):
        self.LoadName = QtWidgets.QFileDialog.getOpenFileName( self, 
			"Open Mount Conf file : ", QtCore.QDir.homePath())
        if self.LoadName[0] == '':
            self.ui.prompt_textbrowser.append("No Load File")
        else:
            self.ui.prompt_textbrowser.append("Load Conf File : "+self.LoadName[0])
            
            dico = gen.read_conf_file(self.LoadName[0])
            
            # General ____________________________________________
            self.ui.board_combo.setCurrentText(dico["board_type"])
            self.ui.mount_combo.setCurrentText(dico["mount_type"])
            self.ui.maxrate_spin.setValue(dico["max_rate"])
            self.ui.auto_sid_tracking_checkBox.setChecked(dico["auto_sid"])
    
            self.ui.axis1_worm_spindouble.setValue(dico["worm1"])
            self.ui.axis1_gear_spindouble.setValue(dico["gear1"])
            self.ui.axis1_motor_spin.setValue(dico["stepper1"])
            self.ui.axis1_micro_combo.setCurrentText(dico["micro1"])
            self.ui.axis1_mod_combo.setCurrentText(dico["slew1"])
            self.ui.axis1_driver_combo.setCurrentText(dico["driver1"])
            self.ui.axis1_reverse_checkBox.setChecked(dico["reverse1"])
            self.ui.axis1_enable_comboBox.setCurrentText(dico["ena1"])
            self.ui.axis1_fault_comboBox.setCurrentText(dico["fault1"])
            
            self.ui.axis2_worm_spindouble.setValue(dico["worm2"])
            self.ui.axis2_gear_spindouble.setValue(dico["gear2"])
            self.ui.axis2_motor_spin.setValue(dico["stepper2"])
            self.ui.axis2_micro_combo.setCurrentText(dico["micro2"])
            self.ui.axis2_mod_combo.setCurrentText(dico["slew2"])
            self.ui.axis2_driver_combo.setCurrentText(dico["driver2"])
            self.ui.axis2_reverse_checkBox.setChecked(dico["reverse2"])
            self.ui.axis2_enable_comboBox.setCurrentText(dico["ena2"])
            self.ui.axis2_fault_comboBox.setCurrentText(dico["fault2"])
            
            # Axis3/2 ____________________________________________
            self.ui.rot_checkBox.setChecked(dico["rot3"])
            self.ui.focus1_checkBox.setChecked(dico["foc1"])
            self.ui.focus2_checkBox.setChecked(dico["foc2"])
            
            self.ui.rot_mrate_spinBox.setValue(dico["rot_rate"])
            self.ui.rot_step_spinBox.setValue(dico["rot_step"])
            self.ui.rot_mode_comboBox.setCurrentText(dico["rot_micro"])
            self.ui.rot_gear1_doubleSpinBox.setValue(dico["rot_gear"])
            self.ui.rot_gear2_doubleSpinBox.setValue(dico["rot_gear_2"])
            self.ui.rot_reverse_checkBox.setChecked(dico["rot_reverse"])
            self.ui.rot_min_angle_doubleSpinBox.setValue(dico["rot_min_degr"])
            self.ui.rot_max_angle_doubleSpinBox.setValue(dico["rot_max_degr"])
            self.ui.rot_di_comboBox.setCurrentText(dico["rot_disable"])
            
            self.ui.focus1_mrate_spinBox.setValue(dico["foc1_rate"])
            self.ui.focus1_micro_doubleSpinBox.setValue(dico["foc1_ratio"])
            self.ui.focus1_reverse_checkBox.setChecked(dico["foc1_reverse"])
            self.ui.focus1_min_mil_doubleSpinBox.setValue(dico["foc1_min_mm"])
            self.ui.focus1_max_mil_doubleSpinBox.setValue(dico["foc1_max_mm"])
            self.ui.focus1_di_comboBox.setCurrentText(dico["focus1_disable"])
            
            self.ui.focus2_mrate_spinBox.setValue(dico["foc2_rate"])
            self.ui.focus2_micro_doubleSpinBox.setValue(dico["foc2_ratio"])
            self.ui.focus2_reverse_checkBox.setChecked(dico["foc2_reverse"])
            self.ui.focus2_min_mil_doubleSpinBox.setValue(dico["foc2_min_mm"])
            self.ui.focus2_max_mil_doubleSpinBox.setValue(dico["foc2_max_mm"])
            self.ui.focus2_di_comboBox.setCurrentText(dico["focus2_disable"])
            
            # option ____________________________________________
            self.ui.serial_baud_comboBox.setCurrentText(dico["baud"])
            self.ui.serial4_baud_comboBox.setCurrentText(dico["baud4"])
            
            self.ui.esp_checkBox.setChecked(dico["esp"])
            
            self.ui.pec_checkBox.setChecked(dico["pec"])
            self.ui.pec_pul_checkBox.setChecked(dico["pec_pul"])
            self.ui.pec_buffer_spin.setValue(dico["pec_buffer"])
            
            self.ui.goto_assist_checkBox.setChecked(dico["goto_assist"])
            
            self.ui.strict_checkBox.setChecked(dico["strict_park"])
            
            self.ui.st4_checkBox.setChecked(dico["st4"])
            self.ui.st4_pul_checkBox.setChecked(dico["st4_pul"])
            self.ui.alternative_st4_checkBox.setChecked(dico["alt_st4"])
            self.ui.hand_checkBox.setChecked(dico["hand"])
            
            self.ui.separate_pulse_checkBox.setChecked(dico["pulse"])
            
            self.ui.guide_time_spinBox.setValue(dico["guide_time"])
            
            self.ui.rtc_combo.setCurrentText(dico["rtc"])
            
            self.ui.pps_checkBox.setChecked(dico["pps"])
            self.ui.pps_pul_checkBox.setChecked(dico["pps_pul"])
            
            self.ui.pec_set_checkBox.setChecked(dico["pec_set"])
            self.ui.analog_pec_spinBox.setValue(dico["analog_pec"])
            self.ui.pec_logic_comboBox.setCurrentText(dico["pec_logic"])
            
            self.ui.limit_checkBox.setChecked(dico["limit"])
            
            self.ui.state_led_checkBox.setChecked(dico["led1"])
            
            self.ui.state_led2_checkBox.setChecked(dico["led2"])
            self.ui.intensity_led2_spinBox.setValue(dico["led2_intensity"])
            self.ui.reticule_checkBox.setChecked(dico["reticule"])
            self.ui.intensity_ret_spinBox.setValue(dico["ret_intensity"])
            
            self.ui.buzzer_checkBox.setChecked(dico["buzzer"])
            self.ui.buzzer_type_comboBox.setCurrentText(dico["buzzer_type"])
            self.ui.freq_spinBox.setValue(dico["freq_sound"])
            self.ui.default_sound_checkBox.setChecked(dico["def_sound"])
            
            self.ui.atmos_checkBox.setChecked(dico["atmos"])
            
            self.ui.save_mer_flip_checkBox.setChecked(dico["mem_flip_mer"])
            
            self.ui.home_pause_checkBox.setChecked(dico["home_pause"])
            
            self.ui.max_rate_save_checkBox.setChecked(dico["mem_max_rate"])
            
            self.ui.degre_accel_doubleSpinBox.setValue(dico["accel"])
            
            self.ui.degre_rapid_stop_doubleSpinBox.setValue(dico["rapid_stop"])
            
            self.ui.backlash_rate_spinBox.setValue(dico["backlash"])
            
            self.ui.off_axis2_checkBox.setChecked(dico["off_axis2"])
            
            self.ui.degre_e_spinBox.setValue(dico["degre_e"])
            self.ui.degre_w_spinBox.setValue(dico["degre_w"])
            
            self.ui.min_dec_spinBox.setValue(dico["min_dec"])
            self.ui.max_dec_spinBox.setValue(dico["max_dec"])
            
            self.ui.pol_limit_spinBox.setValue(dico["pol_limit"])
            
            self.ui.max_az_spinBox.setValue(dico["max_az"])
            
            self.ui.prompt_textbrowser.append("Load Succes") 
            
    def create_conf_file(self,path_name,var):
        file = open(path_name,'w')
        
        for i in range(len(var)):
            file.write(str(var[i])+"\n")
            
        file.close()
        
    
    def main(self):
        self.show()

if __name__ == "__main__":
	import sys
	app = QtWidgets.QApplication(sys.argv)
	myapp = ShipHolderApplication()
	myapp.main()
	sys.exit(app.exec_())

