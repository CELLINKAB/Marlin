pipeline {
    agent none
    stages {
        agent any
        stage('Git Version ') {

            sh 'docker run --rm -v "$(pwd):/repo" gittools/gitversion:5.6.6 /repo /output file'
        
        }
        stage('Building firmwares') {
            matrix {
                    agent {
                    dockerfile true
                    }
                axes {
                    axis {
                        name 'BOARD'
                        values 'MYCORRHIZA_V1_1'
                    }
                }
                stages {
                    stage('Building Firmware') {
                        steps {
                            sh '''
                                git clean -Xdf
                                git status
                                echo "Do Build for ${BOARD}"
                                python3 -m platformio run --environment  ${BOARD}
                            '''
                        }
                    }
                }
                post {
                    always {
                        sh '''
                            cp  .pio/build/${BOARD}/firmware.bin ./${BOARD}-${BUILD_NUMBER}.bin
                                                    '''
                        archiveArtifacts artifacts: " ${BOARD}-${BUILD_NUMBER}.bin"
                        sh '''
                            python3 -m platformio run --target clean --environment ${BOARD}
                        '''
                    // always do this because 'success' is performed after 'always', even if it's listed before..
                    }
                }
            }
        }
    }
}
