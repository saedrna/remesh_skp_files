//
//  ComponentInstance.hpp
//
//  Sketchup C++ Wrapper for C API
//  Copyright (C) 2016  Hidetomo (Tom) Kaneko
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef ComponentInstance_hpp
#define ComponentInstance_hpp

#include <stdio.h>

#include <SketchUpAPI/model/component_instance.h>

#include "Entity.hpp"
#include "Transformation.hpp"
#include "ComponentDefinition.hpp"

namespace CW {

class ComponentInstance :public DrawingElement {
	private:
  SUComponentInstanceRef m_instance;
	
  /** Creates a copy of the ComponentInstance */
	SUComponentInstanceRef copy_reference(const ComponentInstance& other);

  public:
  ComponentInstance(SUComponentInstanceRef instance, bool attached = true);
  
	/** Copy constructor */
  ComponentInstance(const ComponentInstance& other);
  
  /** Destructor */
  ~ComponentInstance();

  /** Copy assignment operator */
  ComponentInstance& operator=(const ComponentInstance& other);
  
  /**
  * Returns the raw SUComponentInstance object.
  */
  SUComponentInstanceRef ref();

  /** Cast to native objects */
  operator SUComponentInstanceRef() const;
  operator SUComponentInstanceRef*();
  
  /**
  * Returns the Transformation object applied to this instance.
  */
  Transformation transformation() const;
  
  /**
  * Set the transformation applied to this Component Instance
  */
  void transformation(const Transformation& tranform);

  /**
  * Returns the Component Definition object of this instance.
  */
  ComponentDefinition definition() const;

  /**
  * Returns the name of this instance.
  */
  String name() const;

  /**
  * Sets the name of this instance.
  */
  void name(const String& string);
  
};

} /* namespace CW */
#endif /* ComponentInstance_hpp */
