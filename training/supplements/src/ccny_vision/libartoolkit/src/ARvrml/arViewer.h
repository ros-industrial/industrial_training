// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 4; -*-
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

#ifndef AR_VRMLINT_H
#define AR_VRMLINT_H

#include <iostream>
#include <fstream>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/utility.hpp>
#include <openvrml/browser.h>
#include <openvrml/gl/viewer.h>
#include <openvrml/bounding_volume.h>
#ifdef _WIN32
#  include <windows.h>
#endif

class arVrmlBrowser : public openvrml::browser {
public:
	arVrmlBrowser();
	
private:
	virtual std::auto_ptr<openvrml::resource_istream>
	do_get_resource(const std::string & uri);
};

class arVrmlViewer : public openvrml::gl::viewer {

public:
  arVrmlViewer();
  ~arVrmlViewer() throw ();

    char             filename[512];
    double           translation[3];
    double           rotation[4];
    double           scale[3];
    bool             internal_light;

    void timerUpdate();
    void redraw();
    void setInternalLight( bool f );

protected:
    virtual void post_redraw();
    virtual void set_cursor(openvrml::gl::viewer::cursor_style c);
    virtual void swap_buffers();
    virtual void set_timer(double);


   virtual void set_viewpoint(const openvrml::vec3f & position,
                                       const openvrml::rotation & orientation,
                                       float fieldOfView,
                                       float avatarSize,
                                       float visibilityLimit);

   virtual viewer::object_t insert_background(const std::vector<float> & groundAngle,
											  const std::vector<openvrml::color> & groundColor,
											  const std::vector<float> & skyAngle,
											  const std::vector<openvrml::color> & skyColor,
											  const openvrml::image & front,
											  const openvrml::image & back,
											  const openvrml::image & left,
											  const openvrml::image & right,
											  const openvrml::image & top,
											  const openvrml::image & bottom);
		
    virtual viewer::object_t insert_dir_light(float ambientIntensity,
                                              float intensity,
                                              const openvrml::color & color,
                                              const openvrml::vec3f & direction);

    virtual viewer::object_t insert_point_light(float ambientIntensity,
                                                const openvrml::vec3f & attenuation,
                                                const openvrml::color & color,
                                                float intensity,
                                                const openvrml::vec3f & location,
                                                float radius);

    virtual viewer::object_t insert_spot_light(float ambientIntensity,
                                               const openvrml::vec3f & attenuation,
                                               float beamWidth,
                                               const openvrml::color & color,
                                               float cutOffAngle,
                                               const openvrml::vec3f & direction,
                                               float intensity,
                                               const openvrml::vec3f & location,
                                               float radius);
    virtual openvrml::bounding_volume::intersection
      intersect_view_volume(const openvrml::bounding_volume & bvolume) const;
};

#endif
