@echo off
echo =====================================
echo  Compilando y subiendo tu robot FTC...
echo =====================================
cd /d "C:\Users\ricay\OneDrive\Documentos\IDR\FTC\RobotProgram2025\FtcRobotController"
echo.
call gradlew assembleDebug
echo.
echo --- Conectando al Control Hub ---
adb connect 192.168.43.1:5555
echo.
echo --- Instalando APK ---
adb install -r "C:\Users\ricay\OneDrive\Documentos\IDR\FTC\RobotProgram2025\FtcRobotController\TeamCode\build\outputs\apk\debug\TeamCode-debug.apk"
echo.
echo --- Listo, Ricardo. Tu robot está actualizado. ---
