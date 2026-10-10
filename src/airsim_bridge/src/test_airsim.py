import airsim

c = airsim.MultirotorClient(ip="127.0.0.1", port=41451)
c.confirmConnection()
print("Connected!")

vehicles = c.listVehicles()
print("vehicles:", vehicles)

for vn in vehicles:
    try:
        c.simGetVehiclePose(vn)
        print(f"simGetVehiclePose('{vn}') OK")
    except Exception as e:
        print(f"simGetVehiclePose('{vn}') failed:", repr(e))
    for ln in ["Lidar", "LidarSensor1", ""]:
        try:
            d = c.getLidarData(ln, vn) if ln else c.getLidarData()
            print(f"getLidarData('{ln}', '{vn}'): {len(d.point_cloud)//3} pts")
        except Exception as e:
            print(f"getLidarData('{ln}', '{vn}') failed:", repr(e))
