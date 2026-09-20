^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2025-06-09)
------------------

1.7.0 (2024-06-12)
------------------

1.6.1 (2024-06-01)
------------------
* Fix build with apple clang (`#27 <https://github.com/open-rmf/rmf_utils/pull/27>`_)
* Contributors: Yadunund

1.6.0 (2023-06-08)
------------------

1.5.0 (2023-06-06)
------------------
* Switch to rst changelogs (`#28 <https://github.com/open-rmf/rmf_utils/pull/28>`_)
* Reusable CI (`#24 <https://github.com/open-rmf/rmf_utils/pull/24>`_)
* Contributors: Esteban Martinena Guerrero, Yadunund

1.4.0 (2022-02-14)
------------------
* Add a class to help with limiting rates of events (`#18 <https://github.com/open-rmf/rmf_utils/pull/18>`_)

1.3.0 (2020-01-19)
------------------
* Adding missing string include for test, only shows up when building with clang (`#9 <https://github.com/open-rmf/rmf_utils/pull/9>`_)
* Add quality declaration documents (`#1 <https://github.com/open-rmf/rmf_utils/pull/1>`_)
* Changed package requirement to QUIET to allow use in non-ROS 2 packages (`#6 <https://github.com/open-rmf/rmf_utils/pull/6>`_)
* install rmf_code_style.cfg in rmf_utils_DIR (`#4 <https://github.com/open-rmf/rmf_utils/pull/4>`_)
* change to catch2 test, uncrustify everything (`#3 <https://github.com/open-rmf/rmf_utils/pull/3>`_)
* Contributors: Aaron Chong, Geoffrey Biggs, ddengster

1.1.0 (2020-09-24)
------------------
* Replace rmf_utils::optional with std::optional (`#177 <https://github.com/osrf/rmf_core/pull/177>`_)
* Foxy Support (`#133 <https://github.com/osrf/rmf_core/pull/133>`_)
* Remove ros2 dependency (`#142 <https://github.com/osrf/rmf_core/pull/142>`_)
* Contributors: Aaron Chong, Grey, Yadu

1.0.0 (2020-06-23)
------------------
* Basic utilities for use in the `rmf_core` packages
    * `impl_ptr` - A smart pointer that helps implement a PIMPL-pattern
    * `clone_ptr` - A smart pointer with cloning semantics (copying the pointer instance will clone the underlying object)
    * `optional` - A stopgap measure to get the features of `std::optional` before C++17 is available
    * `Modular` - A class for comparing integral version numbers that may wrap around due to integer overflow
* Contributors: Grey, Luca Della Vedova, Marco A. Gutiérrez, Michael X. Grey, Morgan Quigley, Yadu, Yadunund
