#[[
Copyright (c) 2015-2021 University of Antwerp, Aloxy NV.

This file is part of Sub-IoT.
See https://github.com/Sub-IoT/Sub-IoT-Stack for further info.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
]]


INCLUDE_DIRECTORIES(
    .
)

EXPORT_GLOBAL_INCLUDE_DIRECTORIES(
    .
)

SET(hts221_SRC
    HTS221_Driver.c
    hal_glue.c
)


#An object library with name '${CHIP_LIBRARY_NAME}' MUST be generated by the CMakeLists.txt file for every chip
ADD_LIBRARY (${CHIP_LIBRARY_NAME} OBJECT ${hts221_SRC})
