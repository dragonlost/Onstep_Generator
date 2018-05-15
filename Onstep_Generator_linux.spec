# -*- mode: python -*-

block_cipher = None


a = Analysis(['main.py'],
             pathex=['/home/developpeur/github/Onstep_Generator'],
             binaries=[],
             datas=[('OnStep_Logo_Medium.png','.')],
             hiddenimports=[],
             hookspath=[],
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)
exe = EXE(pyz,
          a.scripts,
          a.binaries,
          a.zipfiles,
          a.datas,
          name='Onstep_Generator',
          debug=False,
          strip=False,
          upx=True,
          runtime_tmpdir=None,
          console=False , icon='OnStep.ico')
