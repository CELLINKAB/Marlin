pipeline {  
    agent none
    stages {
        stage('Building firmwares') {
            matrix {
                    agent {
                    dockerfile true
                    }
                axes {
                    axis {
                        name 'BOARD'
                        values 'MYCORRHIZA_V1'
                    }
                }
            stages {
                stage('Building Firmware') {
                        steps {
                            sh '''
                                git clean -Xdf
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