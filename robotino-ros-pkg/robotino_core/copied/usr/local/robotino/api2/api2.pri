ROBOTINO_API2_DIR = $$PWD
message(Robotino API2 located in $$ROBOTINO_API2_DIR)

INCLUDEPATH += $$ROBOTINO_API2_DIR/include

win32: {
	CONFIG( debug, debug|release ) {
            LIBS += "$$ROBOTINO_API2_DIR/lib/rec_robotino_api2d.lib"
            DLL = "$$ROBOTINO_API2_DIR/bin/rec_robotino_api2d.dll"
            QMAKE_PRE_LINK += $${QMAKE_COPY} "$$shell_path($${DLL})" debug
        } else {
            LIBS += "$$ROBOTINO_API2_DIR/lib/rec_robotino_api2.lib"
			DLL = "$$ROBOTINO_API2_DIR/bin/rec_robotino_api2.dll"
            QMAKE_PRE_LINK += $${QMAKE_COPY} "$$shell_path($${DLL})" release
        }
}

unix:!mac {
}

#MacOSX and iOS
mac: {
    #MacOSX
    macx: {
        CONFIG( debug, debug|release ) {
            LIBS += $$ROBOTINO_API2_DIR/lib/librec_robotino_api2_clang64_debug.a \
                    $$ROBOTINO_API2_DIR/lib/librec_cv_lt_clang64_debug.a \
                    $$ROBOTINO_API2_DIR/lib/librec_robotino3_fleetcom_clang64_debug.a \
                    $$ROBOTINO_API2_DIR/lib/librec_robotino_rpc_clang64_debug.a \
                    $$ROBOTINO_API2_DIR/lib/librec_rpc_clang64_debug.a
        } else {
            LIBS += $$ROBOTINO_API2_DIR/lib/librec_robotino_api2_clang64_release.a \
                    $$ROBOTINO_API2_DIR/lib/librec_cv_lt_clang64_release.a \
                    $$ROBOTINO_API2_DIR/lib/librec_robotino3_fleetcom_clang64_release.a \
                    $$ROBOTINO_API2_DIR/lib/librec_robotino_rpc_clang64_release.a \
                    $$ROBOTINO_API2_DIR/lib/librec_rpc_clang64_release.a
        }
    }
    #iOS
    !macx: {
        CONFIG( debug, debug|release ) {
            CONFIG( iphoneos, iphoneos|iphonesimulator ) {
                LIBS += $$ROBOTINO_API2_DIR/lib/librec_robotino_api2_iphoneos_debug.a \
                        $$ROBOTINO_API2_DIR/lib/librec_cv_lt_iphoneos_debug.a \
                        $$ROBOTINO_API2_DIR/lib/librec_robotino3_fleetcom_iphoneos_debug.a \
                        $$ROBOTINO_API2_DIR/lib/librec_robotino_rpc_iphoneos_debug.a \
                        $$ROBOTINO_API2_DIR/lib/librec_rpc_iphoneos_debug.a
            } else {
                LIBS += $$ROBOTINO_API2_DIR/lib/librec_robotino_api2_iphonesimulator_debug.a \
                        $$ROBOTINO_API2_DIR/lib/librec_cv_lt_iphonesimulator_debug.a \
                        $$ROBOTINO_API2_DIR/lib/librec_robotino3_fleetcom_iphonesimulator_debug.a \
                        $$ROBOTINO_API2_DIR/lib/librec_robotino_rpc_iphonesimulator_debug.a \
                        $$ROBOTINO_API2_DIR/lib/librec_rpc_iphonesimulator_debug.a
            }
        } else {
            CONFIG( iphoneos, iphoneos|iphonesimulator ) {
                LIBS += $$ROBOTINO_API2_DIR/lib/librec_robotino_api2_iphoneos_release.a \
                        $$ROBOTINO_API2_DIR/lib/librec_cv_lt_iphoneos_release.a \
                        $$ROBOTINO_API2_DIR/lib/librec_robotino3_fleetcom_iphoneos_release.a \
                        $$ROBOTINO_API2_DIR/lib/librec_robotino_rpc_iphoneos_release.a \
                        $$ROBOTINO_API2_DIR/lib/librec_rpc_iphoneos_release.a
            } else {
                LIBS += $$ROBOTINO_API2_DIR/lib/librec_robotino_api2_iphonesimulator_release.a \
                        $$ROBOTINO_API2_DIR/lib/librec_cv_lt_iphonesimulator_release.a \
                        $$ROBOTINO_API2_DIR/lib/librec_robotino3_fleetcom_iphonesimulator_release.a \
                        $$ROBOTINO_API2_DIR/lib/librec_robotino_rpc_iphonesimulator_release.a \
                        $$ROBOTINO_API2_DIR/lib/librec_rpc_iphonesimulator_release.a
            }
        }
    }
}
