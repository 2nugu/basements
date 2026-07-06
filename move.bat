@echo off
set DEST=g:\CPP\Basements\include\basements\core\math
mkdir "%DEST%" 2>nul
move /Y g:\CPP\Basements\include\basements\vec2.h "%DEST%\"
move /Y g:\CPP\Basements\include\basements\vec3.h "%DEST%\"
move /Y g:\CPP\Basements\include\basements\vec4.h "%DEST%\"
move /Y g:\CPP\Basements\include\basements\quaternion.h "%DEST%\"
move /Y g:\CPP\Basements\include\basements\dual_quaternion.h "%DEST%\"
move /Y g:\CPP\Basements\include\basements\matrix3.h "%DEST%\"
move /Y g:\CPP\Basements\include\basements\matrix4.h "%DEST%\"
move /Y g:\CPP\Basements\include\basements\transform.h "%DEST%\"
move /Y g:\CPP\Basements\include\basements\advanced_numerics.h "%DEST%\"
move /Y g:\CPP\Basements\include\basements\extended_metrics.h "%DEST%\"
move /Y g:\CPP\Basements\include\basements\basements.h "%DEST%\"
move /Y g:\CPP\Basements\include\basements\common.h "%DEST%\"
