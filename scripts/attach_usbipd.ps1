param(
    [string]$BusId = "5-2"
)
# Attach the DM USB2CANFD module to WSL.
# If its STATE is "Not shared", run once in an ADMIN PowerShell:
#     usbipd bind --busid <BusId>
usbipd list
usbipd attach --wsl --busid $BusId
