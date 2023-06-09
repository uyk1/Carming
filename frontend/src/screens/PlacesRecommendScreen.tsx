import {useEffect, useRef} from 'react';
import {Dimensions, ScrollView, StyleSheet, Text, View} from 'react-native';
import {useSelector, useDispatch} from 'react-redux';
import {
  ActivityIndicator,
  Avatar,
  IconButton,
  Tooltip,
  useTheme,
} from 'react-native-paper';
import Carousel from 'react-native-snap-carousel-v4';
import Icon from 'react-native-vector-icons/MaterialCommunityIcons';
import SelectDropdown from 'react-native-select-dropdown';
import styled from 'styled-components/native';
import {Place, Category, Tag} from '../types';
import PlaceRecommendCard from '../components/PlaceRecommendCard';
import TagChip from '../components/TagChip';
import {RootState} from '../redux/store';
import {
  addCheckedTag,
  addPlaceToPlaceCart,
  deleteCheckedTag,
  deletePlaceFromPlaceCartById,
  increasePlacePage,
  resetPlacePage,
  selectCategory,
  setPlaceTagList,
} from '../redux/slices/placeSlice';
import {placeApi, useGetPlacesQuery} from '../apis/placeApi';
import {filterTagsByCategory} from '../utils';
import {ALERT_TYPE, Toast} from 'react-native-alert-notification';

interface PlacesRecommendScreenScreenProps {}

const PlacesRecommendScreen: React.FC<
  PlacesRecommendScreenScreenProps
> = ({}) => {
  const theme = useTheme();
  const dispatch = useDispatch();
  const carouselRef = useRef<any>(null);
  const PAGE_SIZE = 10;

  const {regionList} = useSelector((state: RootState) => state.main);
  const {placeCart, placeTagList, checkedTagList, selectedCategory, placePage} =
    useSelector((state: RootState) => state.place);
  const tags = useSelector((state: RootState) => state.tag);

  const {
    data: places,
    error,
    isFetching,
    isError,
    isSuccess,
  } = useGetPlacesQuery({
    regions: regionList,
    category: selectedCategory,
    size: PAGE_SIZE,
    page: placePage,
    tagId: checkedTagList.length > 0 ? checkedTagList[0].id : undefined,
  });

  useEffect(() => {
    if (!isFetching && places && places.length <= (placePage - 1) * PAGE_SIZE) {
      Toast.show({
        type: ALERT_TYPE.WARNING,
        title: 'Info',
        textBody: '더이상 불러올 장소가 없습니다.',
      });
    }
  }, [places, isFetching]);

  // 카테고리 변경시
  useEffect(() => {
    dispatch(setPlaceTagList(filterTagsByCategory(tags, selectedCategory)));
    dispatch(resetPlacePage());
    dispatch(deleteCheckedTag(checkedTagList[0]));
    dispatch(placeApi.util.invalidateTags(['Places']));
  }, [selectedCategory, tags]);

  const tagPressed = (tag: Tag) => {
    dispatch(resetPlacePage());
    if (checkedTagList.includes(tag)) {
      dispatch(deleteCheckedTag(tag));
    } else {
      dispatch(addCheckedTag(tag));
    }
  };

  const placeAddBtnPressed = () => {
    if (places && places.length > 0) {
      const place: Place = places[carouselRef.current._activeItem];
      dispatch(addPlaceToPlaceCart(place));
    }
  };

  const placeCancelBtnPressed = (placeId: number) => {
    dispatch(deletePlaceFromPlaceCartById(placeId));
  };

  const carouselSection = () => {
    if (isFetching) {
      return (
        <ActivityIndicator
          size={'large'}
          animating={true}
          color={theme.colors.onPrimary}
        />
      );
    }
    if (isError || places?.length === 0) {
      return <Text style={{fontSize: 40}}>😭</Text>;
    }
    if (isSuccess) {
      return (
        <Carousel
          style={{flex: 1}}
          layout={'default'}
          vertical={false}
          layoutCardOffset={3}
          ref={carouselRef}
          data={places}
          firstItem={Math.max(0, places.length - 1 - PAGE_SIZE)}
          renderItem={PlaceRecommendCard}
          sliderWidth={screenWidth}
          itemWidth={screenWidth - 80}
          inactiveSlideShift={0}
          useScrollView={true}
          onScrollIndexChanged={index => {
            if (index === places.length - 1) {
              dispatch(increasePlacePage());
            }
          }}
        />
      );
    }
  };

  return (
    <>
      <StyledView style={{marginTop: 10, marginBottom: 20}}>
        <View style={{maxWidth: 120}}>
          <SelectDropdown
            data={Object.keys(Category)}
            defaultValueByIndex={0}
            onSelect={(selectedItem, index) => {
              dispatch(selectCategory(Object.values(Category)[index]));
            }}
            buttonStyle={styles.dropdown2BtnStyle}
            buttonTextStyle={styles.dropdown2BtnTxtStyle}
            renderDropdownIcon={isOpened => {
              return (
                <Icon
                  name={isOpened ? 'chevron-up' : 'chevron-down'}
                  color={'#FFF'}
                  size={18}
                />
              );
            }}
            dropdownIconPosition={'right'}
            dropdownStyle={styles.dropdown2DropdownStyle}
            rowStyle={styles.dropdown2RowStyle}
            rowTextStyle={styles.dropdown2RowTxtStyle}
          />
        </View>
        <ScrollView
          style={styles.tagScrollViewStyle}
          horizontal={true}
          showsHorizontalScrollIndicator={false}>
          {placeTagList.map(tag => {
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
        </ScrollView>
      </StyledView>

      <CenterView>{carouselSection()}</CenterView>

      <StyledView style={{justifyContent: 'center'}}>
        <IconButton
          icon="arrow-down-drop-circle"
          iconColor={theme.colors.background}
          size={35}
          style={{marginVertical: 20}}
          onPress={() => placeAddBtnPressed()}
        />
      </StyledView>
      <StyledView style={{height: 70}}>
        {placeCart?.map(place => {
          return (
            <Tooltip key={place.id} title={place.name} enterTouchDelay={1}>
              <View style={{marginRight: 5}}>
                <Avatar.Image size={50} source={{uri: place.image}} />
                <IconButton
                  style={{position: 'absolute', right: -17, top: -17}}
                  icon="close-circle"
                  iconColor={theme.colors.background}
                  size={15}
                  onPress={() => placeCancelBtnPressed(place.id)}
                />
              </View>
            </Tooltip>
          );
        })}
      </StyledView>
    </>
  );
};

const {width: screenWidth} = Dimensions.get('window');

const StyledView = styled(View)`
  align-items: center;
  flex-direction: row;
  padding-left: 20px;
  padding-right: 20px;
`;

const CenterView = styled(View)`
  align-items: center;
  justify-content: center;
  flex: 1;
`;

const styles = StyleSheet.create({
  dropdown2BtnStyle: {
    width: '90%',
    height: 30,
    backgroundColor: '#444',
    borderRadius: 8,
  },
  dropdown2BtnTxtStyle: {
    color: '#FFF',
    textAlign: 'center',
    fontWeight: 'bold',
    fontSize: 14,
  },
  dropdown2DropdownStyle: {
    backgroundColor: '#444',
    borderBottomLeftRadius: 12,
    borderBottomRightRadius: 12,
  },
  dropdown2RowStyle: {backgroundColor: '#444', borderBottomColor: '#C5C5C5'},
  dropdown2RowTxtStyle: {
    color: '#FFF',
    textAlign: 'center',
    fontWeight: 'bold',
    fontSize: 14,
  },
  tagScrollViewStyle: {
    flexDirection: 'row',
  },
});

export default PlacesRecommendScreen;
