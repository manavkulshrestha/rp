DD(1)                                         User Commands                                         DD(1)

NNAAMMEE
       dd - convert and copy a file

SSYYNNOOPPSSIISS
       dddd [_O_P_E_R_A_N_D]...
       dddd _O_P_T_I_O_N

DDEESSCCRRIIPPTTIIOONN
       Copy a file, converting and formatting according to the operands.

       bs=BYTES
              read and write up to BYTES bytes at a time (default: 512); overrides ibs and obs

       cbs=BYTES
              convert BYTES bytes at a time

       conv=CONVS
              convert the file as per the comma separated symbol list

       count=N
              copy only N input blocks

       ibs=BYTES
              read up to BYTES bytes at a time (default: 512)

       if=FILE
              read from FILE instead of stdin

       iflag=FLAGS
              read as per the comma separated symbol list

       obs=BYTES
              write BYTES bytes at a time (default: 512)

       of=FILE
              write to FILE instead of stdout

       oflag=FLAGS
              write as per the comma separated symbol list

       seek=N skip N obs-sized blocks at start of output

       skip=N skip N ibs-sized blocks at start of input

       status=LEVEL
              The  LEVEL  of  information to print to stderr; 'none' suppresses everything but error mes‐
              sages, 'noxfer' suppresses the final transfer statistics, 'progress' shows periodic  trans‐
              fer statistics

       N  and  BYTES  may  be  followed  by the following multiplicative suffixes: c =1, w =2, b =512, kB
       =1000, K =1024, MB =1000*1000, M =1024*1024, xM =M, GB =1000*1000*1000, G =1024*1024*1024, and  so
       on for T, P, E, Z, Y.

       Each CONV symbol may be:

       ascii  from EBCDIC to ASCII

       ebcdic from ASCII to EBCDIC

       ibm    from ASCII to alternate EBCDIC

       block  pad newline-terminated records with spaces to cbs-size

       unblock
              replace trailing spaces in cbs-size records with newline

       lcase  change upper case to lower case

       ucase  change lower case to upper case

       sparse try to seek rather than write the output for NUL input blocks

       swab   swap every pair of input bytes

       sync   pad every input block with NULs to ibs-size; when used with block or unblock, pad with spa‐
              ces rather than NULs

       excl   fail if the output file already exists

       nocreat
              do not create the output file

       notrunc
              do not truncate the output file

       noerror
              continue after read errors

       fdatasync
              physically write output file data before finishing

       fsync  likewise, but also write metadata

       Each FLAG symbol may be:

       append append mode (makes sense only for output; conv=notrunc suggested)

       direct use direct I/O for data

       directory
              fail unless a directory

       dsync  use synchronized I/O for data

       sync   likewise, but also for metadata

       fullblock
              accumulate full blocks of input (iflag only)

       nonblock
              use non-blocking I/O

       noatime
              do not update access time

       nocache
              Request to drop cache.  See also oflag=sync

       noctty do not assign controlling terminal from file

       nofollow
              do not follow symlinks

       count_bytes
              treat 'count=N' as a byte count (iflag only)

       skip_bytes
              treat 'skip=N' as a byte count (iflag only)

       seek_bytes
              treat 'seek=N' as a byte count (oflag only)

       Sending a USR1 signal to a running 'dd' process makes it print I/O statistics  to  standard  error
       and then resume copying.

       Options are:

       ----hheellpp display this help and exit

       ----vveerrssiioonn
              output version information and exit

AAUUTTHHOORR
       Written by Paul Rubin, David MacKenzie, and Stuart Kemp.

RREEPPOORRTTIINNGG BBUUGGSS
       GNU coreutils online help: <https://www.gnu.org/software/coreutils/>
       Report dd translation bugs to <https://translationproject.org/team/>

CCOOPPYYRRIIGGHHTT
       Copyright  ©  2018  Free  Software  Foundation,  Inc.   License GPLv3+: GNU GPL version 3 or later
       <https://gnu.org/licenses/gpl.html>.
       This is free software: you are free to change and redistribute it.  There is NO WARRANTY,  to  the
       extent permitted by law.

SSEEEE AALLSSOO
       Full documentation at: <https://www.gnu.org/software/coreutils/dd>
       or available locally via: info '(coreutils) dd invocation'

GNU coreutils 8.30                            September 2019                                        DD(1)
