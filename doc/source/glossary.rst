Glossary
========

.. include:: global_substitutions.txt

Glossary of terms used through out this documentation:

.. glossary::

   API
       An API, or Application Programming Interface, is an interface which is provided by an "application", which in this case is usually a shared library or other language appropriate shared resource. API's are made up of files which define a contract between the software using the interface and the software providing the interface. These files typically manifest as header files in C and C++ and as Python files in Python. In either case it is important that API's are grouped and described in documentation and that they are declared as either public or private. Public interfaces are subject to change rules and changes to the public interfaces affect the next version number of the software which provides them.

   client_library
       A client library is an :term:`API` which provides access to the ROS graph using primitive middleware concepts like Topics, Services, and Actions.

   package
       A single unit of software, including source code, build system files, documentation, tests, and other associated resources.

   REP
        ROS Enhancement Proposal. A document which describes an enhancement, standardization, or convention for the ROS community.
        The associated REP approval process allows the community to iterate on a proposal until some consensus has been made, at which point it can be ratified and implemented, at which point it becomes documentation.
        All REP's are viewable from the `REP index <http://www.ros.org/reps/rep-0000.html>`_.

   VCS
       Version Control System, such as CVS, SVN, git, mercurial, etc...

   rclcpp
       The C++ specific :term:`Client Library <client_library>` for ROS. This includes any middleware related API's as well as the related message generation of C++ data structures based on interface definitions like Messages, Services, and Actions.

   repository
       A collection of packages usually managed using a :term:`VCS` like git or mercurial and usually hosted on a site like GitHub or BitBucket.
       In the context of this document repositories usually contain one or more |packages| of one type or another.
