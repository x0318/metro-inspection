Set files = CreateObject("Scripting.FileSystemObject")
Set shell = CreateObject("WScript.Shell")
script = files.BuildPath(files.GetParentFolderName(WScript.ScriptFullName), "scripts\open_inspection_windows.ps1")
command = "powershell.exe -NoProfile -ExecutionPolicy Bypass -File """ & script & """"
code = shell.Run(command, 0, True)
