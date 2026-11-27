<#
.SYNOPSIS
    ESP32-S31 ILI9341  一键编译 + 烧录 + 自检脚本

.DESCRIPTION
    专门用来根治「烧完还是旧程序」的问题。它会：
      1. 自己搭好 IDF 环境（不依赖 VSCode 的 ESP-IDF Terminal）
      2. 检查 build 里的 app bin 是不是比源码旧，是就强制全量重编
      3. 编译，并断言产物时间戳确实更新了
      4. 同时烧 bootloader + 分区表 + 应用（不是只烧 app）
      5. 烧完自动校验 hash
      6. 复位后读串口，打印固件自报的编译时间，让你亲眼确认跑的是新程序

.PARAMETER Port
    串口，默认 COM10

.PARAMETER ProjDir
    工程目录。默认脚本所在目录。换工程就用 -ProjDir 指过去，不用改脚本。

.PARAMETER Chip
    芯片型号，默认自动从 build\project_description.json 读；读不到就用 esp32s31。

.PARAMETER FlashSize
    Flash 容量，默认自动从 sdkconfig 读；读不到就用 16MB。

.PARAMETER AppOffset
    app 分区起始地址，默认 0x10000。新工程改过分区表才需要传。

.PARAMETER Full
    整片擦除 Flash 后再烧。第一次烧新工程、或怀疑 otadata/旧分区表残留时用。

.PARAMETER NObuild
    跳过编译，直接烧现在的 build 产物。

.PARAMETER NoSerial
    烧完不读串口。

.EXAMPLE
    .\flash.ps1                                      # 当前工程
    .\flash.ps1 -ProjDir "E:\新工程" -Port COM12      # 换个工程
    .\flash.ps1 -Full
#>
param(
    [Parameter(Position = 0)]
    [string]$Port = "COM10",
    [string]$ProjDir = "",
    [string]$Chip = "",
    [string]$FlashSize = "",
    [string]$AppOffset = "0x10000",
    [switch]$Full,
    [switch]$NObuild,
    [switch]$NoSerial
)

$ErrorActionPreference = "Continue"

function Step($msg) { Write-Host ""; Write-Host "==> $msg" -ForegroundColor Cyan }
function Ok($msg)   { Write-Host "    [OK] $msg" -ForegroundColor Green }
function Warn($msg) { Write-Host "    [!!] $msg" -ForegroundColor Yellow }
function Die($msg)  { Write-Host ""; Write-Host "==> 失败: $msg" -ForegroundColor Red; exit 1 }

# ---------- 路径 / 自动识别 ----------
# $PSScriptRoot 在 param 默认值阶段还没初始化，必须到这里再取
if (-not $ProjDir) { $ProjDir = $PSScriptRoot }
$Root = "E:\.espressif\master"
if (-not (Test-Path $ProjDir)) { Die "工程目录不存在: $ProjDir" }
$ProjDir  = (Resolve-Path $ProjDir).Path
$BuildDir = Join-Path $ProjDir "build"

# 从 build\project_description.json 自动读芯片型号、工程名、IDF 路径
# （新工程只要先编译过一次就有这个文件；没编译过就退回下面的默认值）
$ChipAuto = ""
$ProjName = ""
$DescFile = Join-Path $BuildDir "project_description.json"
if (Test-Path $DescFile) {
    try {
        $desc = Get-Content $DescFile -Raw -Encoding UTF8 | ConvertFrom-Json
        if ($desc.target)      { $ChipAuto = $desc.target }
        if ($desc.project_name){ $ProjName  = $desc.project_name }
        if ($desc.idf_path -and (Test-Path $desc.idf_path)) {
            $idfRoot = $desc.idf_path
            # json 里记的是 ...\esp-idf，而工具链都装在它的上一级
            if ((Split-Path $idfRoot -Leaf) -ieq "esp-idf") { $idfRoot = Split-Path $idfRoot -Parent }
            $Root = $idfRoot
        }
    } catch {
        Warn "project_description.json 解析失败，用默认参数"
    }
}
if (-not $Chip)     { $Chip = if ($ChipAuto) { $ChipAuto } else { "esp32s31" } }
if (-not $ProjName) { $ProjName = Split-Path $ProjDir -Leaf }

$AppBin  = Join-Path $BuildDir ($ProjName + ".bin")
$BootBin = Join-Path $BuildDir "bootloader\bootloader.bin"
$PartBin = Join-Path $BuildDir "partition_table\partition-table.bin"

# Flash 容量：从 sdkconfig 读
if (-not $FlashSize) {
    $FlashSize = "16MB"
    $sdkFile = Join-Path $ProjDir "sdkconfig"
    if (Test-Path $sdkFile) {
        $m = Select-String -Path $sdkFile -Pattern '^CONFIG_ESPTOOLPY_FLASHSIZE="([^"]+)"' |
             Select-Object -First 1
        if ($m) { $FlashSize = $m.Matches[0].Groups[1].Value }
    }
}

# 工具链：在 $Root\tools 下按版本目录自动找，找不到才用这台机器的已知版本
function Find-ToolDir([string]$rel, [string]$filter) {
    $base = Join-Path $Root $rel
    if (Test-Path $base) {
        $hit = Get-ChildItem -LiteralPath $base -Directory -Filter $filter -ErrorAction SilentlyContinue |
               Sort-Object Name -Descending | Select-Object -First 1
        if ($hit) { return $hit.FullName }
    }
    return ""
}

$PyVenvDir   = Join-Path $Root "tools\python\master\venv"
if (-not (Test-Path $PyVenvDir)) {
    $d = Find-ToolDir "tools\python" "master"
    if ($d) { $PyVenvDir = Join-Path $d "venv" }
}
$Py      = Join-Path $PyVenvDir "Scripts\python.exe"
$IdfPy   = Join-Path $Root "esp-idf\tools\idf.py"

$cmakeVer  = (Find-ToolDir "tools\cmake" "4.*")
if ($cmakeVer) { $cmakeBin = Join-Path $cmakeVer "bin" }
else           { $cmakeBin = Join-Path $Root "tools\cmake\4.0.3\bin" }
$ninjaDir  = (Find-ToolDir "tools\ninja" "*")
if (-not $ninjaDir)  { $ninjaDir  = Join-Path $Root "tools\ninja\1.12.1" }
$riscvDir  = ""
foreach ($d in (Get-ChildItem -LiteralPath (Join-Path $Root "tools\riscv32-esp-elf") -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending)) {
    $cand = Join-Path $d.FullName "riscv32-esp-elf\bin"
    if (Test-Path $cand) { $riscvDir = $cand; break }
}
if (-not $riscvDir) { $riscvDir = Join-Path $Root "tools\riscv32-esp-elf\esp-16.1.0_20260609\riscv32-esp-elf\bin" }
$romDir    = (Find-ToolDir "tools\esp-rom-elfs" "*")
if (-not $romDir)    { $romDir    = Join-Path $Root "tools\esp-rom-elfs\20260528" }

# flash_chunked.py / read_serial.py：优先用新工程里的，没有就用脚本所在目录的
$Chunker = Join-Path $ProjDir "flash_chunked.py"
if (-not (Test-Path $Chunker)) { $Chunker = Join-Path $PSScriptRoot "flash_chunked.py" }
$Reader  = Join-Path $ProjDir "read_serial.py"
if (-not (Test-Path $Reader))  { $Reader  = Join-Path $PSScriptRoot "read_serial.py" }

# ---------- 1. 环境 ----------
Step "搭建 IDF 环境"
if (-not (Test-Path $Py)) { Die "找不到 Python: $Py" }
if (-not (Test-Path $IdfPy)) { Die "找不到 idf.py: $IdfPy" }

$env:IDF_PATH             = Join-Path $Root "esp-idf"
$env:IDF_TOOLS_PATH       = $Root
$env:IDF_PYTHON_ENV_PATH  = $PyVenvDir
$env:ESP_IDF_VERSION      = "6.2.0"
$env:IDF_VERSION          = "6.2.0"
$env:IDF_CCACHE_ENABLE    = "0"
$env:IDF_CONFIGDEP_ENABLE = "0"
$env:ESP_ROM_ELF_DIR      = $romDir

$ccacheDir = (Find-ToolDir "tools\ccache" "*")
if (-not $ccacheDir) { $ccacheDir = Join-Path $Root "tools\ccache\4.12.1\ccache-4.12.1-windows-x86_64" }
$configdepDir = (Find-ToolDir "tools\esp-idf-configdep" "*")
if (-not $configdepDir) { $configdepDir = Join-Path $Root "tools\esp-idf-configdep\0.2.3" }
foreach ($cand in @((Join-Path $configdepDir "*\bin"), (Join-Path $configdepDir "bin"), $configdepDir)) {
    if (Test-Path $cand) { $configdepDir = (Resolve-Path $cand).Path; break }
}
# ccache / configdep 的目录里还嵌了一层版本目录，用通配把它里面的 bin 找出来
foreach ($cand in @((Join-Path $ccacheDir "*\bin"), (Join-Path $ccacheDir "bin"), $ccacheDir)) {
    if (Test-Path $cand) { $ccacheDir = (Resolve-Path $cand).Path; break }
}

$toolPaths = @(
    (Join-Path $PyVenvDir "Scripts"),
    $ccacheDir,
    $cmakeBin,
    $ninjaDir,
    $riscvDir,
    $configdepDir
)
$env:PATH = ($toolPaths -join ";") + ";" + $env:PATH
Set-Location $ProjDir
Ok "工程 = $ProjDir"
Ok "芯片 = $Chip   Flash = $FlashSize   app 偏移 = $AppOffset"
Ok "IDF_PATH = $env:IDF_PATH"

# ---------- 2. 检查产物是否比源码旧 ----------
$newestSrc = Get-ChildItem -Path (Join-Path $ProjDir "main") -Recurse -Include *.c, *.h |
             Sort-Object LastWriteTime -Descending | Select-Object -First 1

$needClean = $false
if ($newestSrc -and (Test-Path $AppBin)) {
    $binTime = (Get-Item $AppBin).LastWriteTime
    if ($binTime -lt $newestSrc.LastWriteTime) {
        Warn "固件比源码旧！（bin $binTime < $($newestSrc.Name) $($newestSrc.LastWriteTime)）"
        Warn "这正是「烧完还是旧程序」的典型症状 —— 强制全量重编"
        $needClean = $true
    } else {
        Ok "产物比源码新（bin $binTime）"
    }
} elseif (-not (Test-Path $AppBin)) {
    Warn "还没有编译产物，直接开始编译"
}

# ---------- 3. 编译 ----------
if (-not $NObuild) {
    if ($needClean) {
        Step "全量清理（把 build 目录改名挪开）"
        if (Test-Path $BuildDir) {
            $stale = Join-Path $ProjDir ("build_stale_" + (Get-Date -Format "yyyyMMdd_HHmmss"))
            try {
                Move-Item -Path $BuildDir -Destination $stale -Force
                Ok "已挪到 $stale"
            } catch {
                Die "挪不动 build 目录：$($_.Exception.Message)"
            }
        }
    }

    Step "编译"
    # --no-configdep: 全量编译时 configdep 会报 Permission denied (13)，关掉
    # --no-ccache   : 少一个中间层，排查问题时更干净
    & $Py $IdfPy -B build --no-configdep --no-ccache build
    if ($LASTEXITCODE -ne 0) { Die "编译失败，见上面报错" }
    Ok "编译完成"
}

if (-not (Test-Path $AppBin)) { Die "没有 $AppBin" }

# ---------- 4. 断言产物确实是新的 ----------
Step "校验产物"
$appItem = Get-Item $AppBin
Write-Host ("    固件: {0}" -f $AppBin)
Write-Host ("    大小: {0:N0} 字节 (0x{1:X})" -f $appItem.Length, $appItem.Length)
Write-Host ("    时间: {0}" -f $appItem.LastWriteTime)
if ($newestSrc -and $appItem.LastWriteTime -lt $newestSrc.LastWriteTime) {
    Die "编译完了产物还是比源码旧，说明 ninja 复用了旧目标。请把 build 目录改名挪开后重跑本脚本。"
}
Ok "产物时间戳晚于所有源码"

# ---------- 5. 烧录 ----------
Step "烧录到 $Port"
Write-Host "    串口只允许一个程序占用 —— 如果你 VSCode 里开着串口监视器，先关掉它" -ForegroundColor DarkGray

$esptoolArgs = @(
    "-m", "esptool",
    "--chip", $Chip,
    "-p", $Port,
    "-b", "460800",
    "--before", "default-reset",
    "--after", "hard-reset",
    "--connect-attempts", "5"
)

if ($Full) {
    Step "整片擦除 Flash（-Full）"
    & $Py @esptoolArgs erase-flash
    if ($LASTEXITCODE -ne 0) { Die "擦除失败，检查串口是否被占用 / 板子是否在下载模式" }
    Ok "擦除完成"
}

# bootloader 和分区表都很小（几十 KB），一次写完从来没有失败过
$writeArgs = $esptoolArgs + @(
    "write-flash",
    "--flash-mode", "dio",
    "--flash-size", $FlashSize,
    "--flash-freq", "80m",
    "0x2000",  $BootBin,
    "0x8000",  $PartBin
)
& $Py @writeArgs
if ($LASTEXITCODE -ne 0) { Die "bootloader / 分区表烧录失败" }
Ok "bootloader + 分区表写入完成"

# 应用固件 3.7MB：一次性写出去会在大块传输时失去响应
# （--no-stub 走 ROM 在 80% 断，stub 模式在传到 1.1MB 压缩块时直接 no response），
# 波特率降到 230400 也一样。所以交给 flash_chunked.py 切成 1MB 一块逐块写。
Step "分块烧录应用固件"
& $Py $Chunker --port $Port --bin $AppBin --chip $Chip --base $AppOffset
if ($LASTEXITCODE -ne 0) { Die "应用固件烧录失败" }
Ok "应用固件烧录完成，每块 hash 已校验"

# ---------- 6. 读串口证明跑的是新固件 ----------
if (-not $NoSerial) {
    Step "读取串口日志（8 秒）"
    Start-Sleep -Milliseconds 300
    if (Test-Path $Reader) {
        & $Py $Reader $Port 8
        Write-Host ""
        Write-Host "    上面日志里找这一行，它的时间应该就是刚才的编译时间：" -ForegroundColor DarkGray
        Write-Host "      APP: CAM SENDER BUILD: <日期> <时间>" -ForegroundColor DarkGray
        Write-Host "    如果时间对不上，说明烧进去的不是这份代码。" -ForegroundColor DarkGray
    } else {
        Warn "找不到 read_serial.py，跳过串口验证"
    }
}

Write-Host ""
Write-Host "全部完成。" -ForegroundColor Green
