/*
	Original version for VRML97-0.8.7 by Hirokazu Kato.
 
	Updated for OpenVRML-0.14.3 by Raphael Grasset.
	-remove vrmlScene since we use reference now.
	-remove culling pb

	Updated for OpenVRML-0.16.1 by Philip Lamb. 2006-11-15.
	- add arVrmlBrowser class and Boost dependencies.
*/
/*
 * 
 * This file is part of ARToolKit.
 * 
 * ARToolKit is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * ARToolKit is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with ARToolKit; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */

#include <iostream>
#include <math.h>
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#include "arViewer.h"

using namespace openvrml;

arVrmlBrowser::arVrmlBrowser(): openvrml::browser(std::cout, std::cerr)
{
}

std::auto_ptr<openvrml::resource_istream>
arVrmlBrowser::do_get_resource(const std::string & uri)
{
	using std::auto_ptr;
	using std::invalid_argument;
	using std::string;
	using openvrml::resource_istream;
	
	class file_resource_istream : public resource_istream {
		std::string url_;
		std::filebuf buf_;
		
	public:
		explicit file_resource_istream(const std::string & path): resource_istream(&this->buf_)
		{
			if (!this->buf_.open(path.c_str(), ios_base::in)) {
				this->setstate(ios_base::failbit);
			}
		}
			
		void url(const std::string & str) throw (std::bad_alloc)
		{
			this->url_ = str;
		}
			
	private:
		virtual const std::string do_url() const throw ()
		{
			return this->url_;
		}
			
		virtual const std::string do_type() const throw ()
		{
			//
			// A real application should use OS facilities for this.  This
			// is a crude hack.
			//
			using std::find;
			using std::string;
			using boost::algorithm::iequals;
			using boost::next;
			string media_type = "application/octet-stream";
			const string::const_reverse_iterator dot_pos =
				find(this->url_.rbegin(), this->url_.rend(), '.');
			if (dot_pos == this->url_.rend()
				|| next(dot_pos.base()) == this->url_.end()) {
				return media_type;
			}
			const string::const_iterator hash_pos =
				find(next(dot_pos.base()), this->url_.end(), '#');
			const string ext(dot_pos.base(), hash_pos);
			if (iequals(ext, "wrl")) {
				media_type = "model/vrml";
			} else if (iequals(ext, "x3dv")) {
				media_type = "model/x3d+vrml";
			} else if (iequals(ext, "png")) {
				media_type = "image/png";
			} else if (iequals(ext, "jpg") || iequals(ext, "jpeg")) {
				media_type = "image/jpeg";
			}
			return media_type;
		}
			
		virtual bool do_data_available() const throw ()
		{
			return !!(*this);
		}
	}; // class file_resource_istream
	
	const string scheme = uri.substr(0, uri.find_first_of(':'));
	if (scheme != "file") {
		throw invalid_argument('\"' + scheme + "\" URI scheme not supported");
	}
	//
	// file://
	//        ^
	// 01234567
	//
	string path = uri.substr(uri.find_first_of('/', 7));
	
	auto_ptr<resource_istream> in(new file_resource_istream(path));
	static_cast<file_resource_istream *>(in.get())->url(uri);
	
	return in;
}


arVrmlViewer::arVrmlViewer() : gl::viewer::viewer()
{
    internal_light = true;

    translation[0] = 0.0;
    translation[1] = 0.0;
    translation[2] = 0.0;

    rotation[0] = 0.0;
    rotation[1] = 0.0;
    rotation[2] = 0.0;
    rotation[3] = 0.0;

    scale[0] = 1.0;
    scale[1] = 1.0;
    scale[2] = 1.0;
}

arVrmlViewer::~arVrmlViewer() throw()
{
  
}

void arVrmlViewer::timerUpdate()
{ 
	this->update(0.0);
}



void arVrmlViewer::setInternalLight(bool flag)
{
    internal_light = flag;
}

void arVrmlViewer::redraw()
{
	//double start = browser::current_time();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated( translation[0], translation[1], translation[2] );
    if (rotation[0] != 0.0) {
        glRotated(rotation[0], rotation[1], rotation[2], rotation[3]);
    }
    glScaled(scale[0], scale[1], scale[2]);
	
#if USE_STENCIL_SHAPE
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 1, 1);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
#endif
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_FOG);          // this is a global attribute
    //glDisable(GL_TEXTURE_2D);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
	
	if (internal_light) {
		if (lit) glEnable(GL_LIGHTING);
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
		glEnable(GL_NORMALIZE);
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);
		glShadeModel(GL_SMOOTH);
		
		for (int i = 0; i < max_lights; ++i) {
			light_info_[i].type = light_unused;
			GLenum light = (GLenum) (GL_LIGHT0 + i);
			glDisable(light);
		}
	}
	
    objects = 0;
    nested_objects = 0;
    sensitive = 0;
	
    browser()->render();
	
	if (internal_light) {
		if (lit) glDisable(GL_LIGHTING);
	}
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    //glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void  arVrmlViewer::post_redraw()
{

}
void  arVrmlViewer::set_cursor(cursor_style c)
{

}

void  arVrmlViewer::swap_buffers()
{

}

void  arVrmlViewer::set_timer(double t)
{

}


void arVrmlViewer::set_viewpoint(const openvrml::vec3f & position,
				 const openvrml::rotation & orientation,
				 float fieldOfView,
				 float avatarSize,
				 float visibilityLimit)
{

}

viewer::object_t arVrmlViewer::insert_background(const std::vector<float> & groundAngle,
												 const std::vector<color> & groundColor,
												 const std::vector<float> & skyAngle,
												 const std::vector<color> & skyColor,
												 const image & front,
												 const image & back,
												 const image & left,
												 const image & right,
												 const image & top,
												 const image & bottom)
{
	return 0;
}

viewer::object_t arVrmlViewer::insert_dir_light(float ambientIntensity,
				  float intensity,
				  const openvrml::color & color,
				  const openvrml::vec3f & direction)
{
    if (internal_light) return gl::viewer::insert_dir_light(ambientIntensity, intensity, color, direction);
    return 0;
}

viewer::object_t arVrmlViewer::insert_point_light(float ambientIntensity,
				    const openvrml::vec3f & attenuation,
				    const openvrml::color & color,
				    float intensity,
				    const openvrml::vec3f & location,
				    float radius)
{
	if (internal_light) return gl::viewer::insert_point_light(ambientIntensity, attenuation, color, intensity, location, radius);

    return 0;
}

 
viewer::object_t arVrmlViewer::insert_spot_light(float ambientIntensity,
				   const openvrml::vec3f & attenuation,
				   float beamWidth,
				   const openvrml::color & color,
				   float cutOffAngle,
				   const openvrml::vec3f & direction,
				   float intensity,
				   const openvrml::vec3f & location,
				   float radius)
{
    if (internal_light) return gl::viewer::insert_spot_light(ambientIntensity, attenuation, beamWidth, color,cutOffAngle, direction, intensity, location, radius);
    return 0;
}

 bounding_volume::intersection
arVrmlViewer::intersect_view_volume(const bounding_volume & bvolume) const
{
  // if( d_cull ) {
  //return  openvrml::viewer::intersect_view_volume(bvolume);
	// }
	// else {
         return bounding_volume::inside;
	//}
}
