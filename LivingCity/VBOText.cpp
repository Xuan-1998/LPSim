//---------------------------------------------------------------------------------------------------------------------
// Copyright 2017, 2018 Purdue University, Ignacio Garcia Dorado, Daniel Aliaga
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the 
// following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the 
// following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
// following disclaimer in the documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote 
// products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, 
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//---------------------------------------------------------------------------------------------------------------------

#include "VBOText.h"


#include "VBORenderManager.h"
#include "qimage.h"
#include <QGLWidget>

namespace LC {

	bool VBOText::initialized=false;
	const static QString texFile="data/textTex.png";
	const static int imgSizeY=128;
	const static QString textToTex("abcdefghijklmnopqrstuvwxyz0123456789");
	static int imgSizeX;
	QHash<QChar,QVector2D> VBOText::charToPos;

	void VBOText::createTextTex(){
		printf(">>createTextTex\n");
		QFont font("Arial", imgSizeY/2);
		QFontMetrics fontMetrics(font);
		// length
		imgSizeX=0;
		for(int cN=0;cN<textToTex.size();cN++){
			imgSizeX+=fontMetrics.width(textToTex.at(cN));
		}
		printf("totalLength %f\n",imgSizeX);
		QPixmap pixmap(imgSizeX, imgSizeY);
		pixmap.fill(Qt::transparent);
		QPainter painter;
		painter.begin(&pixmap);
		painter.setRenderHints(QPainter::HighQualityAntialiasing| QPainter::TextAntialiasing|QPainter::Antialiasing|QPainter::SmoothPixmapTransform);
		//QFont font;
		
		painter.setFont(font);
		painter.setPen(Qt::white);
		const int stroke=3;
		int pixStart=0;
		for(int cN=0;cN<textToTex.size();cN++){
			QChar c=textToTex.at(cN);
			// save to hash
			int width=fontMetrics.width(c);
			charToPos[c]=QVector2D(pixStart+1,pixStart+width+(stroke)*2);
			//paint
			//painter.drawText(pixStart,fontMetrics.ascent(), c);
			//pixStart+=width;
			QPainterPath path;
			path.addText(pixStart+stroke,fontMetrics.ascent(), font, c);
			painter.setPen(QPen(Qt::black, stroke));//, Qt::DashDotLine, Qt::RoundCap));
			painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
			painter.drawPath(path);

			pixStart+=width+(stroke)*2;
		}
		painter.end();
		pixmap.save(texFile);
	}//

	void VBOText::addText(VBORenderManager& rendManager,QString geoName,QString text,QVector3D pt,QVector3D dir,QVector3D dirUp,float height,bool centered){
		if(initialized==false){
			//create text texture
			createTextTex();
			if(rendManager.nameToTexId.contains("text")){
				glDeleteTextures(1,&rendManager.nameToTexId["text"]);
				rendManager.nameToTexId.remove("text");
			}
			rendManager.nameToTexId["text"]=rendManager.loadTexture(texFile);
			imgSizeX=QImage(texFile).width();
			initialized=true;
		}

		std::vector<Vertex> texV;
		texV.resize(text.length()*4);//4 vertex per square
		QVector3D p0,p1,p2,p3;
		p0=pt;//init point
		if(centered==true){
			p0=p0-dirUp*(height/2.0f);//4.0?
			pt=pt-dirUp*(height/2.0f);
		}
		float onePixInM=height/(imgSizeY);
		for(int cN=0;cN<text.size();cN++){
			QChar c=text[cN];
			if(charToPos.contains(c)==false){
                printf("missing letter %c\n",c.toLatin1());
				continue;
			}
			QVector2D lettPos=charToPos[c];
			float letterPix=lettPos.y()-lettPos.x();
			p1=p0+dir*(letterPix*onePixInM);
			p2=p1+dirUp*height;
			p3=p0+dirUp*height;
			texV[cN*4+0]=Vertex(p0,QVector3D(lettPos.x()/imgSizeX,0,0));
			texV[cN*4+1]=Vertex(p1,QVector3D(lettPos.y()/imgSizeX,0,0));
			texV[cN*4+2]=Vertex(p2,QVector3D(lettPos.y()/imgSizeX,1.0f,0));
			texV[cN*4+3]=Vertex(p3,QVector3D(lettPos.x()/imgSizeX,1.0f,0));
			//next loop iter
			p0=p1;
		}
		if(centered==true){
			float centerDir=(p1-pt).length()/2.0f;
			QVector3D center=dir*centerDir;
			for(int pN=0;pN<texV.size();pN++){
				texV[pN].info[0]-=center.x();
				texV[pN].info[1]-=center.y();
				texV[pN].info[2]-=center.z();
			}
		}
		rendManager.addStaticGeometry(geoName,texV,texFile,GL_QUADS,2|mode_AdaptTerrain);// texture not lighting

	}//

	void VBOText::renderText(VBORenderManager& rendManager,QString text,QVector3D pt,QVector3D dir,QVector3D dirUp,float height,bool centered){
		//remove
		rendManager.removeStaticGeometry("text_once",false);
		//add
		addText(rendManager,"text_once",text,pt,dir,dirUp,height,centered);
		//render
		glEnable(GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glBlendEquation(GL_FUNC_ADD);
		rendManager.renderStaticGeometry("text_once");
		glDisable(GL_BLEND);
		//remove
		rendManager.removeStaticGeometry("text_once",false);
	}//

} // namespace LC
