import styled from 'styled-components/native';
import TagChip from '../components/TagChip';
import {Avatar, IconButton, Tooltip, useTheme} from 'react-native-paper';
import {useEffect, useRef, useState} from 'react';
import Carousel from 'react-native-snap-carousel-v4';
import CourseRecommendCard from '../components/CourseRecommendCard';
import {Dimensions, StyleSheet, View} from 'react-native';
import {Place, Tag} from '../types';
import CustomButton from '../components/CustomButton';
import {useDispatch, useSelector} from 'react-redux';
import {RootState} from '../redux/store';

const {width: screenWidth} = Dimensions.get('window');

const CoursesRecommendScreen = () => {
  const theme = useTheme();
  const carouselRef = useRef<any>(null);
  const dispatch = useDispatch();

  const {courseList, courseCart, courseTagList, checkedTagList} = useSelector(
    (state: RootState) => state.course,
  );
  const [carouselIdx, setCarouselIdx] = useState<number>(0);
  const [carouselData, setCarouselData] = useState<any[]>([]);

  useEffect(() => {
    makeActiveMapList(0);
  }, []);

  const tagPressed = (tag: Tag) => {
    // checkedTagList.includes(tagId)
    //   ? checkedTagList.splice(checkedTagList.indexOf(tagId), 1)
    //   : checkedTagList.push(tagId);
    // setCheckedTagIdList([...checkedTagList]);
  };

  const courseAddBtnPressed = () => {
    // const place: Place =
    //   carouselRef.current.props.data[carouselRef.current._activeItem];
    // addPlaceCartItemById(place.id);
  };

  const canclePlaceBtnPressed = (placeId: number) => {
    // const idx = courseCart.map(place => place.id).indexOf(placeId);
    // const cart = [...courseCart];
    // cart.splice(idx, 1);
    // setPlaceCart([...cart]);
  };

  const makeActiveMapList = (index: number) => {
    const tmpList: any[] = courseList.map((course, idx) => {
      return {course: course, isActive: idx === index};
    });
    console.log('tmpList : ', tmpList);
    setCarouselData(tmpList);
  };

  return (
    <>
      <StyledView style={{marginTop: 10, marginBottom: 20}}>
        {courseTagList.map(tag => {
          return (
            <TagChip
              key={tag.id}
              style={{marginLeft: 5}}
              text={tag.name}
              selected={checkedTagList.includes(tag)}
              selectedBackgroundColor={theme.colors.secondary}
              onPress={() => tagPressed(tag)}
            />
          );
        })}
      </StyledView>

      <Carousel
        style={{flex: 1}}
        layout={'default'}
        vertical={false}
        layoutCardOffset={9}
        ref={carouselRef}
        data={carouselData}
        renderItem={CourseRecommendCard}
        sliderWidth={screenWidth}
        itemWidth={screenWidth - 80}
        inactiveSlideShift={0}
        useScrollView={true}
        onScrollIndexChanged={index => {
          console.log('carouselRef', carouselRef.current._activeItem, index);
          makeActiveMapList(index);
        }}
      />
      <StyledView style={{justifyContent: 'center'}}>
        <IconButton
          icon="arrow-down-drop-circle"
          iconColor={theme.colors.background}
          size={35}
          style={{marginVertical: 20}}
          onPress={() => courseAddBtnPressed()}
        />
      </StyledView>
      <StyledView style={{height: 70}}>
        {courseCart.map(place => {
          return (
            <Tooltip key={place.id} title={place.name} enterTouchDelay={1}>
              <View style={{marginRight: 5}}>
                <Avatar.Image size={50} source={{uri: place.image}} />
                <IconButton
                  style={{position: 'absolute', right: -17, top: -17}}
                  icon="close-circle"
                  iconColor={theme.colors.background}
                  size={15}
                  onPress={() => canclePlaceBtnPressed(place.id)}
                />
              </View>
            </Tooltip>
          );
        })}
      </StyledView>
      <StyledView style={{justifyContent: 'center'}}>
        <CustomButton
          text={'선택 완료'}
          buttonStyle={{
            width: 200,
            padding: 14,
            borderRadius: 30,
            backgroundColor: theme.colors.surfaceVariant,
          }}
          textStyle={{fontWeight: 'bold', fontSize: 16, textAlign: 'center'}}
        />
      </StyledView>
    </>
  );
};

const StyledView = styled(View)`
  align-items: center;
  flex-direction: row;
  padding-left: 20px;
  padding-right: 20px;
`;

export default CoursesRecommendScreen;
