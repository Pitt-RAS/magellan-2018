pipeline {
    agent { dockerfile true }
    stages {
        stage('Test') {
            steps {
                sh '''
                . /robot/devel/setup.sh
                catkin_make run_tests
                '''
            }
        }
    }
}
