Modifiziere TfViewerApp.py

installiere
pip3 install pyqt6

Add other requirements here....

To install the package
Go into your 'src' folder of your ros2 installation
Open a terminal
'git clone https://github.com/match-PM/pm_tf_viewer.git'

To build the package (make sure you are in the ros workspace)
'colcon build --packages-select pm_tf_viewer'

To start the Application
'ros2 run pm_tf_viewer'

To change the branch
'git checkout main'
'git checkout leonard_working'

To add changes 
'git add --all'
"git commit - m 'MEANINGFULL MESSAGE'"
'git push'




