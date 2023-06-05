from setuptools import setup

package_name = 'Proyecto'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sebastian',
    maintainer_email='s.guerrero3@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['send = Proyecto.visionSend:main',
                            'reseive = Proyecto.visionReseive:main',
                            'persepcion = Proyecto.servicioVision:main',
                            'navigation = Proyecto.robot_navigation:main',
                            'control = Proyecto.robot_control:main',
                            'recreate = Proyecto.robot_recreate:main',
        ],
    },
)
