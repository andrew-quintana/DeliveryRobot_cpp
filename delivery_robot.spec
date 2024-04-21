# -*- mode: python ; coding: utf-8 -*-
block_cipher = None

a = Analysis(['python/main.py'],
             pathex=['/home/jetbot/DeliveryRobot'],
             binaries=[
                 ('/usr/local/lib/libapriltag.so', '.'),
                 ('/usr/local/lib/libapriltag.so.3', '.'),
                 ('/usr/local/lib/libopencv_core.so', '.'),
                 ('/usr/local/lib/libopencv_imgproc.so', '.'),
                 ('/usr/local/lib/libopencv_highgui.so', '.'),
                 ('/usr/local/lib/libboost_graph.so', '.'),
                 ('/usr/local/lib/libboost_system.so', '.'),
                 ('/usr/local/lib/libboost_filesystem.so', '.'),
                 ('/usr/local/lib/googletest/libgtest.a', '.'),
                 (r'${EIGEN3_LIBRARIES}', '.')
             ],
             datas=[],
             hiddenimports=[],
             hookspath=[],
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher,
             noarchive=False)
pyz = PYZ(a.pure, a.zipped_data,
          cipher=block_cipher)
exe = EXE(pyz,
          a.scripts,
          a.binaries,
          a.zipfiles,
          a.datas,
          [],
          name='delivery_robot',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          upx_exclude=[],
          runtime_tmpdir=None,
          console=True)
