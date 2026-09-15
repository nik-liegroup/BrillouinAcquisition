"""Run the production editor/save methods with fake device/storage, without Qt.

Run from a Visual Studio developer prompt: python test_objective_editor.py
The method bodies are taken from ScaleCalibration.cpp, not reimplemented here.
"""
from pathlib import Path
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def compile_and_run(source, output):
    compiler = shutil.which('cl')
    flags = []
    links = []
    if not compiler:
        # Also support the standard Build Tools installation without a developer shell.
        base = Path(os.environ['ProgramFiles(x86)'])
        versions = sorted((base / 'Microsoft Visual Studio/2022/BuildTools/VC/Tools/MSVC').glob('*'))
        sdk_versions = sorted((base / 'Windows Kits/10/Include').glob('*'))
        vc, sdk = versions[-1], sdk_versions[-1]
        compiler = str(vc / 'bin/Hostx64/x64/cl.exe')
        flags = [f'/I{vc / "include"}', f'/I{sdk / "ucrt"}']
        sdk_lib = base / 'Windows Kits/10/Lib' / sdk.name
        links = ['/link', f'/LIBPATH:{vc / "lib/x64"}',
                 f'/LIBPATH:{sdk_lib / "ucrt/x64"}', f'/LIBPATH:{sdk_lib / "um/x64"}']
    subprocess.run([compiler, '/nologo', '/EHsc', '/std:c++17', '/W4', *flags,
                    f'/I{ROOT}', f'/Fo{output.with_suffix(".obj")}',
                    f'/Fe{output}', str(source), *links], check=True)
    subprocess.run([str(output)], check=True)


def main():
    production = (ROOT / 'BrillouinAcquisition/src/Acquisition/AcquisitionModes/ScaleCalibration.cpp').read_text()
    names = ['isEditingObjective', 'refreshActiveObjectiveForEditing', 'selectObjectiveForEditing',
             'persistPartial', 'saveScaleCalibration', 'saveFovOffsetCalibration']
    methods = []
    for name in names:
        signature = ('bool' if name == 'isEditingObjective' else 'void') + ' ScaleCalibration::' + name + '('
        begin = production.index(signature)
        end = production.index('\n}\n', begin) + 3
        methods.append(production[begin:end])
    harness = r'''
#include <map>
#include <cassert>
#include <iostream>
#include "BrillouinAcquisition/src/Acquisition/AcquisitionModes/ScaleCalibrationHelper.h"
#define emit
struct FakeScanControl {
    int slot = 6;
    std::map<int, ObjectiveCalibrationData> profiles;
    int getActiveObjectiveSlot() const { return slot; }
    ObjectiveCalibrationData getObjectiveCalibration(int id) const { return profiles.at(id); }
    bool hasObjectiveCalibration(int id) const { return profiles.count(id) != 0; }
    ScaleCalibrationData getScaleCalibration() const { return profiles.at(slot); }
    void setObjectiveCalibration(int id, ObjectiveCalibrationData data) { profiles[id] = data; }
};
struct ScaleCalibration {
    FakeScanControl* m_scanControl;
    ObjectiveCalibrationData m_scaleCalibration;
    int m_editingObjectiveSlot = -2;
    std::map<int, std::string> m_objectiveFilePaths;
    std::string m_linkedCalibrationFilePath;
    std::map<std::string, ObjectiveCalibrationData> written;
    int warnings = 0;
    bool isEditingObjective(int slot) const;
    void refreshActiveObjectiveForEditing(bool discardEdits = false);
    void selectObjectiveForEditing(int slot, std::string path, std::string name, double magnification);
    void persistPartial(bool includeScale, bool includeFov);
    void saveScaleCalibration(int expectedSlot);
    void saveFovOffsetCalibration(int expectedSlot);
    void writeLinkedCalibrationFile() { written[m_linkedCalibrationFilePath] = m_scaleCalibration; }
    void s_scaleCalibrationChanged(ScaleCalibrationData) {}
    void s_objectiveCalibrationChanged(ObjectiveCalibrationData) {}
    void s_scaleCalibrationStatus(std::string, std::string) { ++warnings; }
    void s_fovOffsetSaved(int, POINT2, bool, POINT2, bool) {}
};
'''
    harness += '\n'.join(methods)
    harness += r'''
int main() {
    FakeScanControl device;
    for (int slot : { 6, 1 }) {
        ObjectiveCalibrationData c;
        c.objectiveSlot = slot;
        c.objectiveName = slot == 6 ? "10x" : "20x";
        c.pixToMicrometerX = { 0, slot == 6 ? -.47 : -.235 };
        c.pixToMicrometerY = { slot == 6 ? .46 : .23, 0 };
        c.hasFovOffset = true;
        c.fovOffsetUm = slot == 6 ? POINT2{ 0, 0 } : POINT2{ -100, 170 };
        ScaleCalibrationHelper::initializeCalibrationFromPixel(&c);
        device.profiles[slot] = c;
    }
    ScaleCalibration editor;
    editor.m_scanControl = &device;
    editor.m_objectiveFilePaths = { { 6, "10x.h5" }, { 1, "20x.h5" } };
    editor.refreshActiveObjectiveForEditing();
    assert(editor.m_scaleCalibration.pixToMicrometerX.y == -.47);
    // A hardware switch must load both the correct numerical profile and file.
    device.slot = 1;
    editor.refreshActiveObjectiveForEditing();
    assert(editor.m_scaleCalibration.pixToMicrometerX.y == -.235);
    assert(editor.m_scaleCalibration.fovOffsetUm.y == 170);
    assert(editor.m_linkedCalibrationFilePath == "20x.h5");
    // Delayed GUI selection from the old slot cannot redirect that file.
    editor.selectObjectiveForEditing(6, "wrong.h5", "10x", 10);
    assert(editor.m_linkedCalibrationFilePath == "20x.h5");
    editor.saveScaleCalibration(6);
    assert(editor.written.empty() && editor.warnings == 1);
    // A same-slot GUI refresh must preserve an unsaved automated measurement.
    editor.m_scaleCalibration.fovOffsetUm.y = 173;
    editor.selectObjectiveForEditing(1, "20x.h5", "20x", 20);
    assert(editor.m_scaleCalibration.fovOffsetUm.y == 173);
    editor.saveFovOffsetCalibration(1);
    assert(editor.written.at("20x.h5").fovOffsetUm.y == 173);
    assert(device.profiles.at(1).pixToMicrometerX.y == -.235);
    // Save scale commits only scale and cannot alter the reference profile.
    editor.m_scaleCalibration.pixToMicrometerX.y = -.234;
    editor.m_scaleCalibration.fovOffsetUm.y = 999;
    editor.saveScaleCalibration(1);
    assert(device.profiles.at(1).pixToMicrometerX.y == -.234);
    assert(device.profiles.at(1).fovOffsetUm.y == 173);
    assert(device.profiles.at(6).pixToMicrometerX.y == -.47);
    assert(editor.written.count("10x.h5") == 0);
    editor.m_scaleCalibration.pixToMicrometerX = {};
    editor.saveScaleCalibration(1);
    assert(device.profiles.at(1).pixToMicrometerX.y == -.234);
    // Even before the editor receives a switch, an old queued save is rejected.
    device.slot = 6;
    editor.saveFovOffsetCalibration(1);
    assert(editor.written.count("10x.h5") == 0);
    editor.refreshActiveObjectiveForEditing();
    assert(editor.m_scaleCalibration.pixToMicrometerX.y == -.47);
    assert(editor.m_linkedCalibrationFilePath == "10x.h5");
    std::cout << "PASS: production editor selection, stale events, save isolation, invalid scales\n";
}
'''
    output = ROOT / '.tmp/objective-editor-test'
    output.mkdir(parents=True, exist_ok=True)
    source = output / 'editor.cpp'
    source.write_text(harness)
    compile_and_run(source, output / 'editor.exe')
    compile_and_run(ROOT / 'BrillouinAcquisitionUnitTest/ObjectiveGridTransformStandalone.cpp',
                    output / 'transform.exe')


if __name__ == '__main__':
    main()
