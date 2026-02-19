from setuptools import find_packages, setup

package_name = 'px4_kontrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enesturkoglu2',
    maintainer_email='enesturkoglu2@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kalkis = px4_kontrol.kalkis:main',
            'devriye = px4_kontrol.devriye:main',
            'renk_avcisi = px4_kontrol.renk_avcisi:main',
            'konum_kontrol = px4_kontrol.konum_kontrol:main',
           'tam_otonom_inis = px4_kontrol.tam_otonom_inis:main',
        ],
    },
)
