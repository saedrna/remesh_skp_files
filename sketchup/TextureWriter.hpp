//
//  TextureWriter.hpp
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

#ifndef TextureWriter_hpp
#define TextureWriter_hpp

#include <stdio.h>

#include <SketchUpAPI/model/texture_writer.h>

namespace CW {

/*
* TextureWriter wrapper
*/
class TextureWriter {
	private:
  SUTextureWriterRef m_texture_writer;
  
  public:
  TextureWriter();
  TextureWriter(SUTextureWriterRef texture_writer);
  
  operator SUTextureWriterRef() const;
  operator SUTextureWriterRef*() const;
};

}
#endif /* TextureWriter_hpp */
