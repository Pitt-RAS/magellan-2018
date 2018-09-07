pipeline {
    agent any
    stages {
        stage('Submodule Checkout') {
            steps {
                script {
                    sh 'git submodule update --init --recursive'
                }
            }
        }
        stage('Test') {
            agent { dockerfile true }
            steps {
                sh '''
                . /robot/devel/setup.sh
                catkin_make run_tests
                '''
            }
        }
        stage('Build Teensy') {
            agent { dockerfile true }
            steps {
                sh '''
                . /robot/devel/setup.sh
                rosrun rosserial_arduino make_libraries.py /root/Arduino/libraries/
                /robot/src/magellan_firmware/compile.sh
                '''
            }
        }
    }
}
