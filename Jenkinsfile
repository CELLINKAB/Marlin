pipeline {
    agent { label 'ubuntu'}
    stages {
            stage('Git fetch') {
              steps {
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
                                    [$class: 'RelativeTargetDirectory',
                                        relativeTargetDir: 's'
                                    ],
                                    [$class: 'PruneStaleBranch'],
                                    pruneTags(true)
                                ],
                                submoduleCfg: [],
                                userRemoteConfigs: scm.userRemoteConfigs
                            ])
                                sh '''
                                cd s
                                git clean -ffdx
                                git submodule foreach --recursive git clean -ffdx
                            '''
                            }
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
                        archiveArtifacts artifacts: " GitVersion.json"
                        archiveArtifacts artifacts: " version.json"

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
