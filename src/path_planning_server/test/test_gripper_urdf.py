from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


GENERATED_URDF = (
    Path(__file__).parents[1]
    / 'generated'
    / 'dual_piper.urdf'
)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_gripper_model_supports_ten_centimeter_total_opening(side):
    root = ET.parse(GENERATED_URDF).getroot()
    joints = {
        joint.attrib['name']: joint
        for joint in root.findall('joint')
    }

    joint7_limit = joints[f'{side}_joint7'].find('limit').attrib
    joint8_limit = joints[f'{side}_joint8'].find('limit').attrib

    assert float(joint7_limit['lower']) == pytest.approx(0.0)
    assert float(joint7_limit['upper']) == pytest.approx(0.05)
    assert float(joint8_limit['lower']) == pytest.approx(-0.05)
    assert float(joint8_limit['upper']) == pytest.approx(0.0)
