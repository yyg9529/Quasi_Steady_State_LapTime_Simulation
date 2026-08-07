param(
    [Parameter(Mandatory = $true)]
    [string]$InputCatPart,

    [Parameter(Mandatory = $true)]
    [string]$OutputCsv,

    [double]$SampleSpacingM = 2.0,

    [double]$CadToTrackScale = 10.0
)

$ErrorActionPreference = "Stop"
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$fineSpacingMm = 50.0
$joinToleranceMm = 0.01

function Get-Point2D {
    param([object]$Point)

    $coordinates = @(0.0, 0.0)
    $Point.GetCoordinates([ref]$coordinates)
    return [pscustomobject]@{
        X = [double]$coordinates[0]
        Y = [double]$coordinates[1]
    }
}

function Get-Distance {
    param([object]$A, [object]$B)

    $deltaX = [double]$A.X - [double]$B.X
    $deltaY = [double]$A.Y - [double]$B.Y
    return [Math]::Sqrt($deltaX * $deltaX + $deltaY * $deltaY)
}

function Get-CurveSamples {
    param([object]$Curve, [int]$Index)

    $parameterExtents = @(0.0, 0.0)
    $Curve.GetParamExtents([ref]$parameterExtents)
    $parameterStart = [double]$parameterExtents[0]
    $parameterEnd = [double]$parameterExtents[1]

    $startPoint = Get-Point2D $Curve.StartPoint
    $endPoint = Get-Point2D $Curve.EndPoint
    switch ([int]$Curve.GeometricType) {
        3 { $lengthMm = Get-Distance $startPoint $endPoint }
        5 {
            $lengthMm = [Math]::Abs($parameterEnd - $parameterStart) *
                [double]$Curve.Radius
        }
        default { $lengthMm = [Math]::Abs($parameterEnd - $parameterStart) }
    }
    $intervalCount = [Math]::Max(1, [Math]::Ceiling($lengthMm / $fineSpacingMm))
    $points = [System.Collections.Generic.List[object]]::new()
    for ($sampleIndex = 0; $sampleIndex -le $intervalCount; $sampleIndex++) {
        $fraction = $sampleIndex / $intervalCount
        $parameter = $parameterStart +
            $fraction * ($parameterEnd - $parameterStart)
        $coordinates = @(0.0, 0.0)
        $Curve.GetPointAtParam([double]$parameter, [ref]$coordinates)
        $points.Add([pscustomobject]@{
            X = [double]$coordinates[0]
            Y = [double]$coordinates[1]
        })
    }

    return [pscustomobject]@{
        Index = $Index
        Name = [string]$Curve.Name
        Start = $points[0]
        End = $points[$points.Count - 1]
        Points = $points
    }
}

function Format-Number {
    param([double]$Value)
    return $Value.ToString("G17", $culture)
}

function Get-NodeKey {
    param([object]$Point)

    $xKey = [Math]::Round([double]$Point.X / $joinToleranceMm)
    $yKey = [Math]::Round([double]$Point.Y / $joinToleranceMm)
    return "$xKey,$yKey"
}

if (-not (Test-Path -LiteralPath $InputCatPart -PathType Leaf)) {
    throw "CATPart not found: $InputCatPart"
}
if ($SampleSpacingM -le 0) {
    throw "SampleSpacingM must be positive."
}
if ($CadToTrackScale -le 0) {
    throw "CadToTrackScale must be positive."
}

$catia = $null
$document = $null
try {
    $catia = New-Object -ComObject CATIA.Application
    $catia.Visible = $false
    $document = $catia.Documents.Open((Resolve-Path -LiteralPath $InputCatPart).Path)
    $body = $document.Part.Bodies.Item(1)
    $rib = $body.Shapes.Item(1)
    if ([string]$rib.CenterCurveElement.DisplayName -ne
            [string]$rib.CenterCurve.Name) {
        throw "Unexpected rib center curve: $($rib.CenterCurveElement.DisplayName)"
    }

    $centerSketch = $rib.CenterCurve
    $axis = @(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    $centerSketch.GetAbsoluteAxisData([ref]$axis)
    $expectedAxis = @(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    for ($axisIndex = 0; $axisIndex -lt $axis.Count; $axisIndex++) {
        if ([Math]::Abs([double]$axis[$axisIndex] -
                $expectedAxis[$axisIndex]) -gt 1e-9) {
            throw "Center sketch is not on the expected global XY plane."
        }
    }
    $segments = [System.Collections.Generic.List[object]]::new()
    for ($index = 1; $index -le $centerSketch.GeometricElements.Count; $index++) {
        $element = $centerSketch.GeometricElements.Item($index)
        if ($element.Construction -eq $false -and
                [int]$element.GeometricType -in 0, 3, 5) {
            $segments.Add((Get-CurveSamples $element $index))
        }
    }
    if ($segments.Count -lt 3) {
        throw "Too few non-construction center-curve segments: $($segments.Count)"
    }

    $rawSegmentCount = $segments.Count
    do {
        $degree = @{}
        foreach ($segment in $segments) {
            foreach ($point in @($segment.Start, $segment.End)) {
                $key = Get-NodeKey $point
                if (-not $degree.ContainsKey($key)) {
                    $degree[$key] = 0
                }
                $degree[$key]++
            }
        }
        $dangling = @($segments | Where-Object {
            $degree[(Get-NodeKey $_.Start)] -lt 2 -or
                $degree[(Get-NodeKey $_.End)] -lt 2
        })
        foreach ($segment in $dangling) {
            [void]$segments.Remove($segment)
        }
    } while ($dangling.Count -gt 0)
    $prunedSegmentCount = $rawSegmentCount - $segments.Count
    if ($prunedSegmentCount -ne 1) {
        throw "Expected exactly one dangling non-path segment; found $prunedSegmentCount."
    }

    $startOrigin = [pscustomobject]@{ X = 0.0; Y = 0.0 }
    $first = $segments | Where-Object {
        (Get-Distance $_.Start $startOrigin) -le $joinToleranceMm
    } | Sort-Object Index | Select-Object -First 1
    if ($null -eq $first) {
        throw "No center-curve segment starts at the sketch origin."
    }

    $unused = [System.Collections.Generic.List[object]]::new()
    foreach ($segment in $segments) {
        if ($segment.Index -ne $first.Index) {
            $unused.Add($segment)
        }
    }
    $ordered = [System.Collections.Generic.List[object]]::new()
    foreach ($point in $first.Points) {
        $ordered.Add($point)
    }
    $current = $first.End

    while ($unused.Count -gt 0) {
        $matches = @()
        for ($candidateIndex = 0; $candidateIndex -lt $unused.Count;
                $candidateIndex++) {
            $candidate = $unused[$candidateIndex]
            if ((Get-Distance $current $candidate.Start) -le $joinToleranceMm) {
                $matches += [pscustomobject]@{
                    ListIndex = $candidateIndex
                    Reverse = $false
                    SegmentIndex = $candidate.Index
                    SegmentName = $candidate.Name
                }
            }
            if ((Get-Distance $current $candidate.End) -le $joinToleranceMm) {
                $matches += [pscustomobject]@{
                    ListIndex = $candidateIndex
                    Reverse = $true
                    SegmentIndex = $candidate.Index
                    SegmentName = $candidate.Name
                }
            }
        }
        if ($matches.Count -eq 0 -and
                (Get-Distance $current $ordered[0]) -le $joinToleranceMm) {
            break
        }
        if ($matches.Count -ne 1) {
            $candidateText = ($matches | ForEach-Object {
                "$($_.SegmentIndex):$($_.SegmentName):reverse=$($_.Reverse)"
            }) -join ";"
            throw "Center-curve topology is not a unique loop at point $($ordered.Count): $($matches.Count) candidates [$candidateText]."
        }

        $match = $matches[0]
        $next = $unused[$match.ListIndex]
        $unused.RemoveAt($match.ListIndex)
        $nextPoints = @($next.Points)
        if ($match.Reverse) {
            [array]::Reverse($nextPoints)
        }
        for ($pointIndex = 1; $pointIndex -lt $nextPoints.Count; $pointIndex++) {
            $ordered.Add($nextPoints[$pointIndex])
        }
        $current = $nextPoints[$nextPoints.Count - 1]
    }

    if ((Get-Distance $ordered[0] $ordered[$ordered.Count - 1]) -gt
            $joinToleranceMm) {
        throw "Ordered center curve does not close."
    }
    if ($unused.Count -gt 0) {
        throw "Additional closed center-curve segments remain: $($unused.Count)."
    }

    $profileSketch = $rib.Sketch
    $profileX = [System.Collections.Generic.List[double]]::new()
    for ($index = 1; $index -le $profileSketch.GeometricElements.Count;
            $index++) {
        $element = $profileSketch.GeometricElements.Item($index)
        if ($element.Construction -eq $false -and
                [int]$element.GeometricType -eq 3) {
            $profileX.Add((Get-Point2D $element.StartPoint).X)
            $profileX.Add((Get-Point2D $element.EndPoint).X)
        }
    }
    $ribbonWidthM = (($profileX | Measure-Object -Maximum).Maximum -
        ($profileX | Measure-Object -Minimum).Minimum) /
        1000.0 * $CadToTrackScale

    $cumulativeMm = [System.Collections.Generic.List[double]]::new()
    $cumulativeMm.Add(0.0)
    for ($index = 1; $index -lt $ordered.Count; $index++) {
        $cumulativeMm.Add($cumulativeMm[$index - 1] +
            (Get-Distance $ordered[$index - 1] $ordered[$index]))
    }
    $totalLengthM = $cumulativeMm[$cumulativeMm.Count - 1] /
        1000.0 * $CadToTrackScale
    $targetM = [System.Collections.Generic.List[double]]::new()
    for ($distanceM = 0.0; $distanceM -lt $totalLengthM;
            $distanceM += $SampleSpacingM) {
        $targetM.Add($distanceM)
    }
    $targetM.Add($totalLengthM)

    $resampled = [System.Collections.Generic.List[object]]::new()
    $sourceIndex = 0
    $originX = [double]$ordered[0].X
    $originY = [double]$ordered[0].Y
    foreach ($distanceM in $targetM) {
        $distanceMm = $distanceM * 1000.0 / $CadToTrackScale
        while ($sourceIndex + 1 -lt $cumulativeMm.Count -and
                $cumulativeMm[$sourceIndex + 1] -lt $distanceMm) {
            $sourceIndex++
        }
        $s0 = $cumulativeMm[$sourceIndex]
        $s1 = $cumulativeMm[$sourceIndex + 1]
        $fraction = if ($s1 -gt $s0) { ($distanceMm - $s0) / ($s1 - $s0) }
            else { 0.0 }
        $xMm = $ordered[$sourceIndex].X + $fraction *
            ($ordered[$sourceIndex + 1].X - $ordered[$sourceIndex].X)
        $yMm = $ordered[$sourceIndex].Y + $fraction *
            ($ordered[$sourceIndex + 1].Y - $ordered[$sourceIndex].Y)
        $resampled.Add([pscustomobject]@{
            S = [double]$distanceM
            X = ($xMm - $originX) / 1000.0 * $CadToTrackScale
            Y = ($yMm - $originY) / 1000.0 * $CadToTrackScale
            Curvature = 0.0
        })
    }

    $uniqueCount = $resampled.Count - 1
    for ($index = 0; $index -lt $uniqueCount; $index++) {
        $previous = $resampled[($index - 1 + $uniqueCount) % $uniqueCount]
        $currentPoint = $resampled[$index]
        $nextPoint = $resampled[($index + 1) % $uniqueCount]
        $abX = $currentPoint.X - $previous.X
        $abY = $currentPoint.Y - $previous.Y
        $bcX = $nextPoint.X - $currentPoint.X
        $bcY = $nextPoint.Y - $currentPoint.Y
        $acX = $nextPoint.X - $previous.X
        $acY = $nextPoint.Y - $previous.Y
        $abLength = [Math]::Sqrt($abX * $abX + $abY * $abY)
        $bcLength = [Math]::Sqrt($bcX * $bcX + $bcY * $bcY)
        $acLength = [Math]::Sqrt($acX * $acX + $acY * $acY)
        $denominator = $abLength * $bcLength * $acLength
        if ($denominator -gt 0) {
            $currentPoint.Curvature = 2.0 * ($abX * $bcY - $abY * $bcX) /
                $denominator
        }
    }
    $resampled[$uniqueCount].Curvature = $resampled[0].Curvature

    $outputDirectory = Split-Path -Parent $OutputCsv
    if ($outputDirectory) {
        [System.IO.Directory]::CreateDirectory($outputDirectory) | Out-Null
    }
    $lines = [System.Collections.Generic.List[string]]::new()
    $lines.Add("s_m,x_m,y_m,curvature_1_m,track_width_m")
    foreach ($point in $resampled) {
        $lines.Add((@(
            Format-Number $point.S
            Format-Number $point.X
            Format-Number $point.Y
            Format-Number $point.Curvature
            Format-Number $ribbonWidthM
        ) -join ","))
    }
    [System.IO.File]::WriteAllLines($OutputCsv, $lines,
        [System.Text.UTF8Encoding]::new($false))

    [pscustomobject]@{
        CenterCurve = [string]$rib.CenterCurveElement.DisplayName
        RawSegmentCount = $rawSegmentCount
        PrunedDanglingSegments = $prunedSegmentCount
        SegmentCount = $segments.Count
        IgnoredClosedSegments = $unused.Count
        NodeCountIncludingClosure = $resampled.Count
        CadToTrackScale = $CadToTrackScale
        LengthM = $totalLengthM
        RibbonWidthM = $ribbonWidthM
        OutputCsv = [System.IO.Path]::GetFullPath($OutputCsv)
    }
}
finally {
    if ($null -ne $document) {
        try { $document.Close() } catch {}
    }
    if ($null -ne $catia) {
        try { $catia.Quit() } catch {}
    }
}
