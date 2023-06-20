pipeline {
    agent { label 'onpremise-node' }
    stages {
            stage('Git') {
            steps {
                cleanWs()
                checkout([
                                $class: 'GitSCM',
                                branches: scm.branches,
                                doGenerateSubmoduleConfigurations: false,
                                extensions: [
                                    [$class: 'SubmoduleOption',
                                        disableSubmodules: false,
                                        parentCredentials: true,
                                        recursiveSubmodules: true,
                                        reference: '',
                                        trackingSubmodules: false,
                                        threads: 4,
                                    ],
                                    pruneTags(false)
                                ],
                                submoduleCfg: [],
                                userRemoteConfigs: scm.userRemoteConfigs
                            ])
                sh '''
                                git clean -ffdx
                                git submodule foreach --recursive git clean -ffdx
                            '''
            }
            }
        stage('Build') {
            matrix {
                    agent {
                        dockerfile {
                            filename 'Dockerfile'
                            label 'onpremise-node'
                      }
                    }
                axes {
                    axis {
                        name 'BOARD'
                        values 'STM32H743Vx_btt', 'STM32H723Vx_btt'
                    }
                }
                stages {
                    stage('Building Firmware') {
                        steps {
                            sh '''
                                git clean -Xdf

                                git status
                                echo "Do Build for ${BOARD}"
                                cd ./Firmware/Marlin-bugfix-2.0.9.3.x/
                                python3 -m platformio run --environment  ${BOARD}
                            '''
                        }
                    }
                }
                post {
                    always {
                        sh '''
                            cp  ./Firmware/Marlin-bugfix-2.0.9.3.x/.pio/build/${BOARD}/firmware.bin ./${BOARD}-${BUILD_NUMBER}.bin
                                                    '''
                        archiveArtifacts artifacts: " ${BOARD}-${BUILD_NUMBER}.bin"
                        archiveArtifacts artifacts: ' Firmware/Marlin-bugfix-2.0.9.3.x/GitVersion.json'
                        archiveArtifacts artifacts: ' Firmware/Marlin-bugfix-2.0.9.3.x/version.json'

                        sh '''
                            cd ./Firmware/Marlin-bugfix-2.0.9.3.x/
                            python3 -m platformio run --target clean --environment ${BOARD}
                        '''
                    // always do this because 'success' is performed after 'always', even if it's listed before..
                    }
                }
            }
        }
    }
}
