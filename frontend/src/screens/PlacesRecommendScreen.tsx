import {SafeAreaView} from 'react-native-safe-area-context';
import LinearGradient from 'react-native-linear-gradient';
import styled from 'styled-components/native';
import CommonChip from '../components/CommonChip';
import {IconButton, useTheme} from 'react-native-paper';
import {SegmentedButtons} from 'react-native-paper';
import {useEffect, useRef, useState} from 'react';
import Carousel from 'react-native-snap-carousel-v4';
import RecommendCard from '../components/RecommendCard';
import {Dimensions, Text, View} from 'react-native';

const {width: screenWidth} = Dimensions.get('window');

const PlacesRecommendScreen = () => {
  const theme = useTheme();
  const carouselRef = useRef(null);

  const [placeList, setPlaceList] = useState<Place[]>([]);
  const [tagList, setTagList] = useState<Tag[]>([]);
  const [recommendType, setRecommendType] = useState<string>('');
  const [checkedTagIdList, setCheckedTagIdList] = useState<number[]>([]);

  const recommendTypeChangeButtons = [
    {
      value: '0',
      label: '장소',
      icon: 'map-marker',
      checkedColor: 'white',
      uncheckedColor: 'white',
      style: {
        borderRadius: 10,
        backgroundColor:
          recommendType === '0' ? theme.colors.primary : theme.colors.shadow,
      },
    },
    {
      value: '1',
      label: '코스',
      icon: 'routes',
      checkedColor: 'white',
      uncheckedColor: 'white',
      style: {
        borderRadius: 10,
        backgroundColor:
          recommendType === '1' ? theme.colors.primary : theme.colors.shadow,
      },
    },
  ];

  const tags: Tag[] = [
    {
      id: 0,
      text: '맛있는',
    },
    {
      id: 1,
      text: '청결한',
    },
    {
      id: 2,
      text: '유명한',
    },
  ];

  const places: Place[] = [
    {
      id: 0,
      color: 'yellow',
      title: 'title1',
      content: 'content1',
      imageUrl: 'https://i.imgur.com/UYiroysl.jpg',
    },
    {
      id: 1,
      color: 'red',
      title: 'title2',
      content: 'content2',
      imageUrl: 'https://i.imgur.com/UPrs1EWl.jpg',
    },
    {
      id: 2,
      color: 'blue',
      title: 'title3',
      content: 'content3',
      imageUrl: 'https://i.imgur.com/MABUbpDl.jpg',
    },
    {
      id: 3,
      color: 'green',
      title: 'title4',
      content: 'content4',
      imageUrl: 'https://i.imgur.com/KZsmUi2l.jpg',
    },
  ];

  useEffect(() => {
    setPlaceList(places);
    setTagList(tags);
  }, []);

  const tagPressed = (tagId: number) => {
    checkedTagIdList.includes(tagId)
      ? checkedTagIdList.splice(checkedTagIdList.indexOf(tagId), 1)
      : checkedTagIdList.push(tagId);
    setCheckedTagIdList([...checkedTagIdList]);
  };

  return (
    <GradientBackground colors={['#70558e7a', '#df94c283', '#ffbdc1b0']}>
      <SafeAreaView style={{flex: 1}}>
        <StyledView style={{justifyContent: 'space-between'}}>
          <SegmentedButtons
            style={{width: 200}}
            value={recommendType}
            onValueChange={setRecommendType}
            buttons={recommendTypeChangeButtons}
          />
          <IconButton
            icon="home"
            size={30}
            onPress={() => {
              console.log('hello');
            }}
          />
        </StyledView>
        <StyledView style={{marginTop: 10, marginBottom: 20}}>
          {tagList.map(element => {
            return (
              <CommonChip
                key={element.id}
                style={{marginLeft: 5}}
                text={element.text}
                selected={checkedTagIdList.includes(element.id)}
                selectedBackgroundColor={theme.colors.secondary}
                onPress={() => tagPressed(element.id)}
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
          data={placeList}
          renderItem={RecommendCard}
          sliderWidth={screenWidth}
          itemWidth={screenWidth - 60}
          inactiveSlideShift={0}
          useScrollView={true}
        />
        <Text style={{height: 150, backgroundColor: 'yellow'}}></Text>
      </SafeAreaView>
    </GradientBackground>
  );
};

type Tag = {
  id: number;
  text: string;
};

type Place = {
  id: number;
  color: string;
  title: string;
  content: string;
  imageUrl: string;
};

const StyledView = styled(View)`
  align-items: center;
  flex-direction: row;
  padding-left: 20px;
  padding-right: 20px;
`;

const GradientBackground = styled(LinearGradient)`
  flex: 1;
  padding-top: 20px;
  padding-bottom: 20px;
`;

export default PlacesRecommendScreen;
