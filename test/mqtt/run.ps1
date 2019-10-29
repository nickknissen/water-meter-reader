function Get-ScriptDirectory {
    Split-Path -parent $PSCommandPath
}

$currentDir = Get-ScriptDirectory

docker run -it -p 0.0.0.0:11883:1883 -p 0.0.0.0:9001:9001 -v $currentDir\mosquitto.conf:/mosquitto/config/mosquitto.conf -v $currentDir\passwd:/etc/mosquitto/passwd eclipse-mosquitto