import React from 'react';
import {View} from 'react-native';
import styled from 'styled-components';
import {Course} from '../types';
import CustomMapView from './CustomMapView';

interface CourseRecommendCardProps {
  item: {course: Course; isActive: boolean};
  index: number;
  onPress?: () => void;
}

const CourseRecommendCard: React.FC<CourseRecommendCardProps> = ({item}) => {
  const {course, isActive} = item;
  return (
    <CardContainer pointerEvents="none">
      {isActive ? (
        <CustomMapView viewStyle={{flex: 1}} course={course} />
      ) : (
        <View style={{flex: 1, backgroundColor: 'black'}}></View>
      )}
    </CardContainer>
  );
};

const CardContainer = styled(View)`
  flex: 1;
  border-radius: 5px;
  overflow: hidden;
`;

export default CourseRecommendCard;
