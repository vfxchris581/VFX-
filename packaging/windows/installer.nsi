; Windows installer for HD (64-bit). Built by CI:
;   makensis /DVERSION=0.2.0 /DSRCDIR=dist\HD /DOUTFILE=dist\HD-Windows-Setup.exe packaging\windows\installer.nsi
; Use absolute paths: makensis resolves relative paths from this script's folder.
; SRCDIR must contain HD.exe, hdc.exe, README.md and the examples folder.

Unicode true
!include "MUI2.nsh"
!include "x64.nsh"

!ifndef VERSION
  !define VERSION "0.0.0"
!endif
!ifndef SRCDIR
  !define SRCDIR "dist\HD"
!endif
!ifndef OUTFILE
  !define OUTFILE "HD-Windows-Setup.exe"
!endif

!define APPNAME "HD"
!define COMPANY "vfxchris581"
!define UNINSTKEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APPNAME}"

Name "${APPNAME} ${VERSION}"
OutFile "${OUTFILE}"
InstallDir "$PROGRAMFILES64\${APPNAME}"
InstallDirRegKey HKLM "Software\${COMPANY}\${APPNAME}" "InstallDir"
RequestExecutionLevel admin
SetCompressor /SOLID lzma

VIProductVersion "${VERSION}.0"
VIAddVersionKey "ProductName" "${APPNAME}"
VIAddVersionKey "FileDescription" "${APPNAME} procedural 3D and VFX"
VIAddVersionKey "FileVersion" "${VERSION}"
VIAddVersionKey "ProductVersion" "${VERSION}"
VIAddVersionKey "LegalCopyright" "${COMPANY}"

!define MUI_ABORTWARNING
!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_COMPONENTS
!insertmacro MUI_PAGE_INSTFILES
!define MUI_FINISHPAGE_RUN "$INSTDIR\HD.exe"
!insertmacro MUI_PAGE_FINISH
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_LANGUAGE "English"

Function .onInit
  ${IfNot} ${RunningX64}
    MessageBox MB_ICONSTOP "${APPNAME} requires 64-bit Windows 10 or 11."
    Abort
  ${EndIf}
  SetRegView 64
FunctionEnd

Section "${APPNAME} (required)" SecMain
  SectionIn RO
  SetOutPath "$INSTDIR"
  File "${SRCDIR}\HD.exe"
  File "${SRCDIR}\hdc.exe"
  File "${SRCDIR}\README.md"
  SetOutPath "$INSTDIR\examples"
  File /r "${SRCDIR}\examples\*.*"
  SetOutPath "$INSTDIR"

  WriteRegStr HKLM "Software\${COMPANY}\${APPNAME}" "InstallDir" "$INSTDIR"
  WriteUninstaller "$INSTDIR\Uninstall.exe"
  WriteRegStr HKLM "${UNINSTKEY}" "DisplayName" "${APPNAME}"
  WriteRegStr HKLM "${UNINSTKEY}" "DisplayVersion" "${VERSION}"
  WriteRegStr HKLM "${UNINSTKEY}" "Publisher" "${COMPANY}"
  WriteRegStr HKLM "${UNINSTKEY}" "DisplayIcon" "$INSTDIR\HD.exe"
  WriteRegStr HKLM "${UNINSTKEY}" "InstallLocation" "$INSTDIR"
  WriteRegStr HKLM "${UNINSTKEY}" "UninstallString" '"$INSTDIR\Uninstall.exe"'
  WriteRegDWORD HKLM "${UNINSTKEY}" "NoModify" 1
  WriteRegDWORD HKLM "${UNINSTKEY}" "NoRepair" 1

  CreateDirectory "$SMPROGRAMS\${APPNAME}"
  CreateShortcut "$SMPROGRAMS\${APPNAME}\${APPNAME}.lnk" "$INSTDIR\HD.exe"
  CreateShortcut "$SMPROGRAMS\${APPNAME}\Uninstall ${APPNAME}.lnk" "$INSTDIR\Uninstall.exe"
SectionEnd

Section "Desktop shortcut" SecDesktop
  CreateShortcut "$DESKTOP\${APPNAME}.lnk" "$INSTDIR\HD.exe"
SectionEnd

Section "Open .hdg scenes with ${APPNAME}" SecAssoc
  WriteRegStr HKCR ".hdg" "" "HD.Scene"
  WriteRegStr HKCR "HD.Scene" "" "HD Scene"
  WriteRegStr HKCR "HD.Scene\DefaultIcon" "" "$INSTDIR\HD.exe,0"
  WriteRegStr HKCR "HD.Scene\shell\open\command" "" '"$INSTDIR\HD.exe" "%1"'
SectionEnd

Section "Uninstall"
  SetRegView 64
  Delete "$INSTDIR\HD.exe"
  Delete "$INSTDIR\hdc.exe"
  Delete "$INSTDIR\README.md"
  Delete "$INSTDIR\Uninstall.exe"
  RMDir /r "$INSTDIR\examples"
  RMDir "$INSTDIR"
  Delete "$SMPROGRAMS\${APPNAME}\${APPNAME}.lnk"
  Delete "$SMPROGRAMS\${APPNAME}\Uninstall ${APPNAME}.lnk"
  RMDir "$SMPROGRAMS\${APPNAME}"
  Delete "$DESKTOP\${APPNAME}.lnk"
  ReadRegStr $0 HKCR "HD.Scene\shell\open\command" ""
  StrCmp $0 '"$INSTDIR\HD.exe" "%1"' 0 +3
    DeleteRegKey HKCR "HD.Scene"
    DeleteRegKey HKCR ".hdg"
  DeleteRegKey HKLM "${UNINSTKEY}"
  DeleteRegKey HKLM "Software\${COMPANY}\${APPNAME}"
SectionEnd
