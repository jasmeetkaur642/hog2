//
//  Graphics2D.h
//  IJCAI 2016 Demo
//
//  Created by Nathan Sturtevant on 7/10/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#ifndef Graphics2D_h
#define Graphics2D_h

#include <stdio.h>
#include "GLUtil.h" // TODO: needs to be renamed, if data structures are to be more widely re-used

namespace Graphics2D {

	struct rect {
		double left, top, right, bottom;
	};
	
	struct point2d {
		double x, y;
	};
	
	void FrameRect(rect r, rgbColor c);
	void FillRect(rect r, rgbColor c);
	void FrameCircle(rect r, rgbColor c);
	void FillCircle(rect r, rgbColor c);
	
	void DrawLine(point2d start, point2d end, double lineWidth, rgbColor c);
	void DrawText(const char *text, point2d location, rgbColor c, double height);

}

#endif /* Graphics2D_h */
