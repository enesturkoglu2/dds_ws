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
            
           'qr_kamera_beyni = px4_kontrol.qr_kamera_beyni:main',
           'tam_suru_ajani = px4_kontrol.tam_suru_ajani:main',
           
        ],
    },
)
