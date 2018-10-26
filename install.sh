################################################
# Under GPL license
#     https://www.gnu.org/licenses/gpl.html
# Authors:	Sébastien Durand
# 			
# On Sept 02 2018
# V0.1
################################################
#!/bin/bash -i

sudo add-apt-repository ppa:nilarimogard/webupd8 -y
sudo apt-get update
sudo apt-get -y install python3 python3-pyqt5 yad

if generator_path=`yad --file-selection \
					--directory \
					--center \
					--title="Select OnStep_Generator Folder" \
					--text="Select OnStep_Generator Folder"`
then
	cp $generator_path/temp_Onstep_Generator.desktop.temp $generator_path/Onstep_Generator.desktop
	sed -i -e "s=/LOCAL_FOLDER=$generator_path=" ./Onstep_Generator.desktop
	echo -e "Icon update"

	sudo mv $generator_path/Onstep_Generator.desktop /usr/share/applications/
	echo -e "Propagator Icon was created on the Program list"
	echo -e "OnStep Generator Installation Complete"
else
	echo "exit"
fi
