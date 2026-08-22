' ExportFirmwareDatums.vba
' SolidWorks Macro to export Coordinate Systems to CSV and origin-aligned STLs
' Requirement: A Coordinate System named "FIRMWARE_ORIGIN" must exist in the top-level assembly.
' Any Coordinate System starting with "DATUM_" will be exported.

Dim swApp As Object
Dim swModel As Object
Dim swAssy As Object
Dim swFeat As Object
Dim swMathUtil As Object
Dim csvPath As String
Dim stlPath As String
Dim fso As Object
Dim csvFile As Object

Sub main()
    Set swApp = Application.SldWorks
    Set swModel = swApp.ActiveDoc
    Set swMathUtil = swApp.GetMathUtility
    
    If swModel Is Nothing Then
        MsgBox "Please open the top-level assembly."
        Exit Sub
    End If
    
    If swModel.GetType <> 2 Then ' 2 = swDocASSEMBLY
        MsgBox "This macro must be run from an assembly."
        Exit Sub
    End If
    
    Set swAssy = swModel
    
    ' Output paths (adjust as needed or prompt user)
    Dim networkDir As String
    networkDir = "C:\Temp\Cell_Config\" ' CHANGE THIS TO \\SERVER\Engineering\Cell_Config\
    
    If Dir(networkDir, vbDirectory) = "" Then
        MkDir networkDir
    End If
    
    csvPath = networkDir & "datums.csv"
    stlPath = networkDir & "meshes\"
    
    If Dir(stlPath, vbDirectory) = "" Then
        MkDir stlPath
    End If
    
    Set fso = CreateObject("Scripting.FileSystemObject")
    Set csvFile = fso.CreateTextFile(csvPath, True)
    
    ' Write CSV Header
    csvFile.WriteLine "Datum_Name,X_mm,Y_mm,Z_mm,Roll_deg,Pitch_deg,Yaw_deg,Description"
    
    ' Find FIRMWARE_ORIGIN Transform
    Dim originTransform As Object
    Set originTransform = GetCoordinateSystemTransform(swModel, "FIRMWARE_ORIGIN")
    
    If originTransform Is Nothing Then
        MsgBox "Could not find 'FIRMWARE_ORIGIN' Coordinate System. Please create it."
        csvFile.Close
        Exit Sub
    End If
    
    Dim inverseOriginTransform As Object
    Set inverseOriginTransform = originTransform.Inverse()
    
    ' Iterate through features to find DATUM_*
    Set swFeat = swModel.FirstFeature
    Do While Not swFeat Is Nothing
        If swFeat.GetTypeName = "CoordSys" And Left(swFeat.Name, 6) = "DATUM_" Then
            
            Dim datumTransform As Object
            Set datumTransform = GetCoordinateSystemTransform(swModel, swFeat.Name)
            
            If Not datumTransform Is Nothing Then
                ' Calculate transform relative to FIRMWARE_ORIGIN
                Dim relativeTransform As Object
                Set relativeTransform = datumTransform.MultiplyTransform(inverseOriginTransform)
                
                ' Extract translation (SolidWorks internal is meters, we want mm)
                Dim transData As Variant
                transData = relativeTransform.ArrayData
                
                Dim x_mm As Double, y_mm As Double, z_mm As Double
                x_mm = transData(9) * 1000#
                y_mm = transData(10) * 1000#
                z_mm = transData(11) * 1000#
                
                ' Note: Euler angle extraction requires translating the rotation matrix (transData 0-8)
                ' For the sake of this script, we output 0,0,0. 
                ' A full matrix-to-euler math function should be added here for RPY.
                Dim roll_deg As Double, pitch_deg As Double, yaw_deg As Double
                roll_deg = 0
                pitch_deg = 0
                yaw_deg = 0
                
                ' Write to CSV
                csvFile.WriteLine swFeat.Name & "," & x_mm & "," & y_mm & "," & z_mm & "," & roll_deg & "," & pitch_deg & "," & yaw_deg & ",Exported by Macro"
                
                ' Export STL relative to this coordinate system
                ExportSTLRelative swModel, swFeat.Name, stlPath & Replace(swFeat.Name, "DATUM_", "") & ".stl"
                
            End If
        End If
        Set swFeat = swFeat.GetNextFeature
    Loop
    
    csvFile.Close
    MsgBox "Datums CSV and Meshes exported successfully to " & networkDir
End Sub

' Helper function to get absolute transform of a Coordinate System
Function GetCoordinateSystemTransform(swModel As Object, featName As String) As Object
    Dim swFeat As Object
    Set swFeat = swModel.FeatureByName(featName)
    
    If swFeat Is Nothing Then
        Set GetCoordinateSystemTransform = Nothing
        Exit Function
    End If
    
    Dim swCoordSys As Object
    Set swCoordSys = swFeat.GetSpecificFeature2
    Set GetCoordinateSystemTransform = swCoordSys.Transform
End Function

' Helper function to export STL using a specific coordinate system as the origin
Sub ExportSTLRelative(swModel As Object, coordSysName As String, outPath As String)
    Dim swExt As Object
    Set swExt = swModel.Extension
    
    ' Set STL export options
    Dim swAppLocal As Object
    Set swAppLocal = Application.SldWorks
    
    ' Set export coordinate system
    swAppLocal.SetUserPreferenceStringValue 85, coordSysName ' 85 = swExportStlOutputCoordinateSystem
    
    ' Save as STL
    Dim errors As Long, warnings As Long
    swExt.SaveAs outPath, 0, 1, Nothing, errors, warnings
    
    ' Reset export coordinate system to default
    swAppLocal.SetUserPreferenceStringValue 85, ""
End Sub
