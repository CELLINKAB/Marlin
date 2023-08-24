pipeline {
    agent { label 'onpremise-node' }
    environment {
        ART_NAME = "${BRANCH_NAME}-${env.BUILD_NUMBER}"
        ART_NAME_NOSLASH = ART_NAME.replaceAll("/", "-")
    }
    stages {
            stage('Git fetch') {
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
        stage('Building firmwares') {
            matrix {
                    agent {
                        dockerfile {
                            filename 'Dockerfile'
                            label 'onpremise-node'
                      }
                    }
                axes {
                    axis {
                        name 'DEVICE'
                        values 'Exocyte', 'Foton'
                    }
                    axis {
                        name 'BOARD'
                        values 'MYCORRHIZA_V1_1', 'STM32H743Vx_btt', 'STM32H723Vx_btt'
                    }
                }
                excludes {
                    exclude {
                        axis {
                            name 'DEVICE'
                            values 'Exocyte'
                        }
                        axis {
                            name 'BOARD'
                            values 'STM32H743Vx_btt', 'STM32H723Vx_btt'
                        }
                    }
                    exclude {
                        axis {
                            name 'DEVICE'
                            values 'Foton'
                        }
                        axis {
                            name 'BOARD'
                            values 'MYCORRHIZA_V1_1'
                        }
                    }
                }
                stages {
                    stage('Building Firmware ${DEVICE}') {
                        steps {
                            sh '''
                                git clean -Xdf
                                git status
                                 python3 -m  platformio run --target clean
                                echo "Using config for ${DEVICE}"
                                cp -f config/${DEVICE}_Configuration.h Marlin/Configuration.h
                                cp -f config/${DEVICE}_Configuration_adv.h Marlin/Configuration_adv.h
                                echo "Do Build for ${BOARD}"
                                python3 -m platformio run --environment  ${BOARD}  -a "--build_tag=${ART_NAME_NOSLASH}"
                            '''
                        }
                    }
                }
                post {
                    always {
                        sh '''
                            cp  .pio/build/${BOARD}/firmware.bin ./${BOARD}-${ART_NAME_NOSLASH}.bin
                                                    '''
                        archiveArtifacts artifacts: " ${BOARD}-${ART_NAME_NOSLASH}.bin"
                        archiveArtifacts artifacts: ' GitVersion.json'
                        archiveArtifacts artifacts: ' version.json'

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
