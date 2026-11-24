#define MyAppName "QDevice x-IMU3 for QTM"
#define MyAppVersion "0.1.2"
#define MyAppPublisher "Technosport"
#define MyAppURL "https://github.com/Technosport/QDevice-xio-ximu3"

[Setup]
AppId={{C9F3E72A-9D41-4A8E-B51F-2E7B1C409110}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
DefaultDirName={autopf64}\Qualisys\Qualisys Track Manager\Input_Devices
DisableDirPage=no
DirExistsWarning=no
DisableProgramGroupPage=yes
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
PrivilegesRequired=admin
OutputDir=..\dist
OutputBaseFilename=QDevice-xIMU3-Setup-v{#MyAppVersion}
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
LicenseFile=..\LICENSE

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
Name: "french"; MessagesFile: "compiler:Languages\French.isl"

[Files]
Source: "..\build\Release\qdevice_ximu3.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\assets\qdevice_ximu3.png"; DestDir: "{app}"; Flags: ignoreversion
