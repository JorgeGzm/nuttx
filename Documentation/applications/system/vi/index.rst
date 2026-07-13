================================
``vi`` VI Work-Alike Text Editor
================================

Description
-----------

The ``vi`` application is a small work-alike of the classic UNIX ``vi``
text editor, allowing files to be created and edited directly on the
target from any VT100-compatible terminal (the typical serial console
client, telnet session, or the NSH console over USB).

It supports the most common vi commands and modes:

* Command mode (``ESC``): cursor movement, delete, yank/paste and
  command repeat
* Insert mode: ``i``, ``I``, ``a``, ``A``, ``o``, ``O``
* Replace: ``r`` (single character) and ``R`` (replace mode)
* Search: ``/`` (forward), ``?`` (backward) and ``f`` (find in line)
* Colon commands: ``:w``, ``:q``, ``:wq``, ``:q!`` and friends

Usage
-----

Basic Syntax
^^^^^^^^^^^^

.. code-block:: bash

   vi [-c <columns>] [-r <rows>] [<filename>]

Options:

================ =========================================================
``<filename>``   Optional name of the file to open
``-c <columns>`` Width of the display in columns (default
                 ``CONFIG_SYSTEM_VI_COLS``)
``-r <rows>``    Height of the display in rows (default
                 ``CONFIG_SYSTEM_VI_ROWS``)
``-h``           Show usage and exit
================ =========================================================

The editor cannot query the terminal for its size, so if the defaults
do not match the terminal, pass the real geometry on the command line,
e.g. for a typical 80x24 terminal:

.. code-block:: bash

   nsh> vi -c 80 -r 24 /tmp/notes.txt

Example
^^^^^^^

Creating a file, editing and checking the result:

.. code-block:: bash

   nsh> vi /tmp/hello.sh
   [type i to enter insert mode, add the text, ESC, then :wq]
   nsh> cat /tmp/hello.sh
   echo hello from vi
   nsh> source /tmp/hello.sh
   hello from vi

Configuration
-------------

**CONFIG_SYSTEM_VI** (selects ``CONFIG_SYSTEM_TERMCURSES`` automatically)

Options:

* **CONFIG_SYSTEM_VI_COLS** - Default display width in columns
  (default 64)
* **CONFIG_SYSTEM_VI_ROWS** - Default display height in rows
  (default 16)
* **CONFIG_SYSTEM_VI_INCLUDE_COMMAND_REPEAT** - Enable the command
  repeat feature
* **CONFIG_SYSTEM_VI_STACKSIZE** - Stack size of the vi task
  (default 2048)
* **CONFIG_SYSTEM_VI_PRIORITY** - Priority of the vi task
  (default 100)
* **CONFIG_SYSTEM_VI_DEBUGLEVEL** - Debug output level (0-2)

Limitations
-----------

* The terminal size is not auto-detected; use ``-c``/``-r`` when the
  configured defaults do not match the terminal.
* A writable file system is required to save files, e.g. tmpfs
  (``CONFIG_FS_TMPFS`` plus ``mount -t tmpfs /tmp`` if the board does
  not mount it at boot), a mounted SD card or a flash file system.
  Without one, ``:w`` fails and ``:q`` will then refuse to exit
  ("No write since last change"); use ``:q!`` to quit discarding the
  changes.
