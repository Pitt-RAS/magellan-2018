node {
    try {
        stage('Pre-build') {
            checkout scm
            sh 'git submodule update --init --recursive'
        }

        stage 'Build'
        def image = docker.build "magellan-2018-${BUILD_TAG.toLowerCase()}"

        stage('Test') {
            image.inside {
                sh '''
                . /opt/magellan-deps/devel/setup.sh
                . /robot/devel/setup.sh
                catkin_make
                catkin_make run_tests
                '''
            }
        }

        stage 'Teensy Image'
        docker.build("magellan-2018-teensy-${BUILD_TAG.toLowerCase()}", "-f Dockerfile.teensy-deploy .")

        stage('Build Teensy') {
            image.inside {
                sh '''
                . /opt/magellan-deps/devel/setup.sh
                . /robot/devel/setup.sh
                rosrun rosserial_arduino make_libraries.py /root/Arduino/libraries/
                /robot/src/magellan_firmware/compile.sh
                '''
            }
        }
    }
    catch (ex) {
        currentBuild.result = 'FAILURE'
    }
    finally {
        stage('Cleanup') {
            cleanWs()
            sh "docker rmi magellan-2018-${BUILD_TAG.toLowerCase()}"
            sh "docker rmi magellan-2018-teensy-${BUILD_TAG.toLowerCase()}"
        }
    }
}
