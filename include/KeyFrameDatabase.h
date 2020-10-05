/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "Map.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/access.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/split_member.hpp>

#include<mutex>


namespace ORB_SLAM3
{

class KeyFrame;
class Frame;
class Map;


class KeyFrameDatabase
{
    private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        boost::serialization::split_member(ar , *this , version);
    }

    template<class Archive>
    void save(Archive& ar , const unsigned int version) const //注意使用 const
    {
        int size = mvInvertedFile.size();
        int size2 = 0;
        ar & size;
        for(vector< list<KeyFrame*> >::const_iterator vit = mvInvertedFile.begin() ; vit != mvInvertedFile.end() ; ++vit)
        {
          ar & (*vit);
          // size2 = (*vit).size();
          // ar & size2;
          // for(list<KeyFrame*>::const_iterator lit = (*vit).begin() ; lit != (*vit).end() ; ++lit )
          // {
          //   ar & (*(*lit));
          // }
        }

        ar & mvBackupInvertedFileId;
    }

    template<class Archive>
    void load(Archive& ar , const unsigned int version)
    {
      int size , size2;
      ar & size;
      for(int i = 0 ; i < size ; ++i)
      {
        // ar & size2;
        list<KeyFrame*> tmp_list;
        ar & tmp_list;

        // for(int j = 0 ; j < size2 ; ++j)
        // {
        //   KeyFrame *pKeyFrame = new KeyFrame();
        //   ar & (*pKeyFrame);
        //   tmp_list.push_back(pKeyFrame);
        // }
        mvInvertedFile.push_back(tmp_list);
      }
        ar & mvBackupInvertedFileId;
    }



public:
   KeyFrameDatabase(){}
   KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();
   void clearMap(Map* pMap);

   // Loop Detection(DEPRECATED)
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Loop and Merge Detection
   void DetectCandidates(KeyFrame* pKF, float minScore,vector<KeyFrame*>& vpLoopCand, vector<KeyFrame*>& vpMergeCand);
   void DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nMinWords);
   void DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F, Map* pMap);

   void PreSave();
   void PostLoad(map<long unsigned int, KeyFrame*> mpKFid);
   void SetORBVocabulary(ORBVocabulary* pORBVoc);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector< list<KeyFrame*> > mvInvertedFile;

  // For save relation without pointer, this is necessary for save/load function
  std::vector< list<long unsigned int> > mvBackupInvertedFileId;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
