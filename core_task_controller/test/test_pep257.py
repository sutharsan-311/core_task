# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # This codebase uses one-line-summary docstrings (D212, already allowed by
    # the ament convention) with Google-style "Args:" sections. The default
    # pydocstyle set also enables the conflicting D213 and the section-format
    # checks, which would flag nearly every docstring. Ignore that convention
    # cluster rather than reformat every docstring in the project.
    rc = main(argv=[
        '.', 'test', '--add-ignore',
        'D205', 'D209', 'D213', 'D400', 'D402', 'D403',
        'D406', 'D407', 'D413', 'D415',
    ])
    assert rc == 0, 'Found code style errors / warnings'
